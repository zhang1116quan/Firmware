/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::navigation_and_controllers)
{
	updateParams();
}

EKF2Selector::~EKF2Selector()
{
	ScheduleClear();
}

bool EKF2Selector::init()
{
	ScheduleDelayed(100_ms);

	return true;
}

void EKF2Selector::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// re-schedule as watchdog timeout
	ScheduleDelayed(20_ms);

	// update combined test ratio for all estimators
	for (int i = 0; i < MAX_INSTANCES; i++) {

		estimator_status_s estimator_status;

		if (_instance[i].estimator_status_sub.update(&estimator_status)) {

			_instance[i].timestamp = estimator_status.timestamp;

			_instance[i].combined_test_ratio = estimator_status.vel_test_ratio
							   + estimator_status.pos_test_ratio
							   + estimator_status.hgt_test_ratio;

			_instance[i].tilt_align = estimator_status.control_mode_flags & (1 << estimator_status_s::CS_TILT_ALIGN);
			_instance[i].yaw_align = estimator_status.control_mode_flags & (1 << estimator_status_s::CS_YAW_ALIGN);

			_instance[i].filter_fault_flags = estimator_status.filter_fault_flags;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
			}
		}
	}

	// TODO: handle switchover
	//   - don't switch until absolutely necessary
	//   - handle reset counter and deltas
	//   - check all fault flags
	//   - publish sensor_selection

	// find highest test ratio and register callback
	uint8_t best_ekf_instance = UINT8_MAX;
	float best_test_ratio = FLT_MAX;

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (hrt_elapsed_time(&_instance[i].timestamp) < 20_ms) {
			if (_instance[i].tilt_align && _instance[i].yaw_align) {
				if (_instance[i].combined_test_ratio < best_test_ratio) {
					best_test_ratio = _instance[i].combined_test_ratio;
					best_ekf_instance = i;
				}
			}
		}
	}

	if (best_ekf_instance != _selected_instance) {

		bool force_reselect = false;

		// critical error if primary has timed out
		if ((_selected_instance == UINT8_MAX) || (hrt_elapsed_time(&_instance[_selected_instance].timestamp) > 50_ms)) {
			force_reselect = true;
		}

		// only switch if current primary has a fault and current best does not
		if (force_reselect ||
		    (_instance[_selected_instance].filter_fault_flags && !_instance[best_ekf_instance].filter_fault_flags)
		   ) {

			// switch callback registration
			if (_selected_instance != UINT8_MAX) {
				_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
				_instance[_selected_instance].estimator_status_sub.unregisterCallback();
			}

			_instance[best_ekf_instance].estimator_attitude_sub.registerCallback();
			_instance[best_ekf_instance].estimator_status_sub.registerCallback();

			if (_selected_instance != UINT8_MAX) {
				PX4_ERR("primary EKF changed %d -> %d", _selected_instance, best_ekf_instance);
			}

			_selected_instance = best_ekf_instance;


			// update sensor_selection immediately
			estimator_sensor_bias_s bias;

			if (_instance[_selected_instance].estimator_sensor_bias_sub.copy(&bias)) {
				sensor_selection_s sensor_selection{};
				sensor_selection.accel_device_id = bias.accel_device_id;
				sensor_selection.gyro_device_id = bias.gyro_device_id;
				sensor_selection.timestamp = hrt_absolute_time();
				_sensor_selection_pub.publish(sensor_selection);
			}

			// handle resets on change

			// vehicle_attitude: quat_reset_counter
			vehicle_attitude_s attitude_new;

			if (_instance[_selected_instance].estimator_attitude_sub.copy(&attitude_new)) {
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude_new.q} - Quatf{_attitude_last.q};

				// save new estimator_attitude
				_attitude_last = attitude_new;

				attitude_new.quat_reset_counter = _quat_reset_counter;
				_delta_q_reset.copyTo(attitude_new.delta_q_reset);

				// publish new vehicle_attitude immediately
				_vehicle_attitude_pub.publish(attitude_new);
			}

			// vehicle_local_position: xy_reset_counter, z_reset_counter, vxy_reset_counter, vz_reset_counter
			vehicle_local_position_s local_position_new;

			if (_instance[_selected_instance].estimator_local_position_sub.copy(&local_position_new)) {
				++_xy_reset_counter;
				++_z_reset_counter;
				++_vxy_reset_counter;
				++_vz_reset_counter;

				_delta_xy = Vector2f{local_position_new.delta_xy} - Vector2f{_local_position_last.delta_xy};
				_delta_z = local_position_new.delta_z - _local_position_last.delta_z;
				_delta_xy = Vector2f{local_position_new.delta_xy} - Vector2f{_local_position_last.delta_xy};
				_delta_vz = local_position_new.delta_vz - _local_position_last.delta_vz;

				// save new estimator_local_position
				_local_position_last = local_position_new;

				local_position_new.xy_reset_counter = _xy_reset_counter;
				local_position_new.z_reset_counter = _z_reset_counter;
				local_position_new.vxy_reset_counter = _vxy_reset_counter;
				local_position_new.vz_reset_counter = _vz_reset_counter;
				_delta_xy.copyTo(local_position_new.delta_xy);
				_delta_vxy.copyTo(local_position_new.delta_vxy);
				local_position_new.delta_z = _delta_z;
				local_position_new.delta_vz = _delta_vz;

				// publish new vehicle_local_position immediately
				_vehicle_local_position_pub.publish(local_position_new);
			}

			// vehicle_global_position: lat_lon_reset_counter, alt_reset_counter
			vehicle_global_position_s global_position_new;

			if (_instance[_selected_instance].estimator_global_position_sub.copy(&global_position_new)) {
				++_alt_reset_counter;
				++_lat_lon_reset_counter;

				_delta_alt = global_position_new.delta_alt - _global_position_last.delta_alt;

				// save new estimator_global_position
				_global_position_last = global_position_new;

				global_position_new.alt_reset_counter = _alt_reset_counter;
				global_position_new.delta_alt = _delta_alt;

				// publish new vehicle_global_position immediately
				_vehicle_global_position_pub.publish(_global_position_last);
			}
		}
	}

	if (_selected_instance != UINT8_MAX) {

		// vehicle_attitude
		vehicle_attitude_s attitude;

		if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {

			if (attitude.quat_reset_counter > _attitude_last.quat_reset_counter) {
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};
			}

			// save latest estimator_attitude
			_attitude_last = attitude;

			// update with total estimator resets
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			_vehicle_attitude_pub.publish(attitude);
		}

		// vehicle_local_position
		vehicle_local_position_s local_position;

		if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {

			// XY reset
			if (local_position.xy_reset_counter > _local_position_last.xy_reset_counter) {
				++_xy_reset_counter;
				_delta_xy = Vector2f{local_position.delta_xy};
			}

			// Z reset
			if (local_position.z_reset_counter > _local_position_last.z_reset_counter) {
				++_z_reset_counter;
				_delta_z = local_position.delta_z;
			}

			// VXY reset
			if (local_position.vxy_reset_counter > _local_position_last.vxy_reset_counter) {
				++_vxy_reset_counter;
				_delta_vxy = Vector2f{local_position.delta_vxy};
			}

			// VZ reset
			if (local_position.vz_reset_counter > _local_position_last.vz_reset_counter) {
				++_vz_reset_counter;
				_delta_z = local_position.delta_vz;
			}

			// save new estimator_local_position
			_local_position_last = local_position;

			// update with total estimator resets
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			_delta_xy.copyTo(local_position.delta_xy);
			_delta_vxy.copyTo(local_position.delta_vxy);
			local_position.delta_z = _delta_z;
			local_position.delta_vz = _delta_vz;

			_vehicle_local_position_pub.publish(local_position);
		}

		// vehicle_global_position
		vehicle_global_position_s global_position;

		if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {

			// Z reset
			if (global_position.alt_reset_counter > _global_position_last.alt_reset_counter) {
				++_alt_reset_counter;
				_delta_alt = global_position.delta_alt;
			}

			// save new estimator_global_position
			_global_position_last = global_position;

			// update with total estimator resets
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt;

			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

int EKF2Selector::print_status()
{
	PX4_INFO("available instances: %d", _available_instances);
	PX4_INFO("selected instance: %d", _selected_instance);

	for (const auto &inst : _instance) {
		PX4_INFO("instance: %d (%.6f seconds ago), tilt align: %d, yaw align: %d, combined test ratio: %.6f", inst.instance,
			 hrt_elapsed_time(&inst.timestamp) / 1e6, inst.tilt_align, inst.yaw_align, (double)inst.combined_test_ratio);
	}

	return 0;
}

int EKF2Selector::task_spawn(int argc, char *argv[])
{
	EKF2Selector *instance = new EKF2Selector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int EKF2Selector::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EKF2Selector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2_selector", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ekf2_selector_main(int argc, char *argv[])
{
	return EKF2Selector::main(argc, argv);
}
