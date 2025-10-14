/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "state_sharing.hpp"
#include <px4_platform_common/events.h>


using namespace time_literals;

// Number of tries for uORB callback registration
constexpr int kNumRegisterTries = 3;

StateSharing::StateSharing() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::state_sharing),
	_predictions(this),
	_publisher_state_sharing(this, px4::wq_configurations::state_sharing)
{
}

StateSharing::~StateSharing()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool StateSharing::init()
{
	// Execute Run() on vehicle global position publication
	for (int i = 0; i < kNumRegisterTries; i++) {
		if (!_vehicle_local_position_sub.registerCallback()) {
			PX4_ERR("callback local pos registration failed");
			px4_usleep(1000);
			continue;
		}

		if (!_mission_command_sub.registerCallback()) {
			PX4_ERR("callback mission command registration failed");
			px4_usleep(1000);
			continue;
		}

		events::send(events::ID("state_sharing_start"), events::Log::Info, "[STATE_SHARING]: started!");
		_predictions.init();
		return true;
	}

	events::send(events::ID("state_sharing_initialization"),
		     events::Log::Error, "[STATE_SHARING]: Register uORB callbacks failed, state sharing didn't start!");
	return false;
}

state_sharing_msg_s StateSharing::getStateSharing()
{
	if (_param_use_predictions.get() && _predictions.enabled()) {
		_predictions.getPrediction2D(
			_state_sharing.global_position_lat,
			_state_sharing.global_position_lon
		);
	}

	return _state_sharing;
}

bool StateSharing::isFirstTimePublish() const
{
	return _first_time_publish;
}

void StateSharing::setFirstTimePublish(const bool &first_time_publish)
{
	_first_time_publish = first_time_publish;
}

bool StateSharing::shouldPublishOutgoingState() const
{
	return _param_use_predictions.get() && _predictions.getCurrentPredictionMethod() == LINEAR_PREDICTION;
}

void StateSharing::Run()
{
	if (should_exit()) {
		_publisher_state_sharing.ScheduleClear();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_mission_command_sub.updated()) {
		mission_command_s mission_command;

		if (_mission_command_sub.copy(&mission_command)) {
			if (mission_command.command == mission_command_s::MISSION_START) {
				PX4_DEBUG("Received mission command [%f] %d",
					  getRealTimeNs() / 1e9,
					  mission_command.command);

				if (!_start) {
					_start = true;
					_first_time_publish = true;
					_publisher_state_sharing.ScheduleOnInterval(
						(double)_param_sharing_period.get() * 1e6,
						0.0
					);
					snprintf(_state_sharing.frame_id, sizeof(_state_sharing.frame_id),
						 "%ld", (long)_param_ident.get());
				}
			}

			if (mission_command.command == mission_command_s::MISSION_END) {
				if (_start) {
					_start = false;
					_publisher_state_sharing.ScheduleClear();
				}
			}

			if (mission_command.command == mission_command_s::CHANGE_PARAMS) {
				ArgParser args(mission_command.args);
				args.printArguments();
				setParameter(args, "state_publishing_dt", "SHARING_PERIOD", _param_sharing_period.get());
				setParameter(args, "ident", "IDENT", (int)_param_ident.get());
				setParameter(args, "use_predictions", "USE_PREDICTIONS", (int)_param_use_predictions.get());
				setParameter(args, "num_timeslots", "NUM_TIMESLOTS", (int)0);
				setParameter(args, "prediction_method", "PREDICT_METHOD", (int)0);
				setParameter(args, "prediction_speed_method", "PRED_SPD_METHOD", (int)0);
			}
		}
	}

	if (!_start) {
		// Check if parameters have changed
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			updateParams(); // update module parameters (in DEFINE_PARAMETERS)
			_predictions.updateParameters();
		}

	} else {
		if (_vehicle_global_position_sub.updated()) {
			vehicle_global_position_s vehicle_global_position;

			if (_vehicle_global_position_sub.copy(&vehicle_global_position)) {
				_state_sharing.global_position_lon = vehicle_global_position.lon;
				_state_sharing.global_position_lat = vehicle_global_position.lat;
				_state_sharing.global_position_alt = vehicle_global_position.alt;
			}
		}

		if (_vehicle_odometry_sub.updated()) {
			vehicle_odometry_s vehicle_odometry;

			if (_vehicle_odometry_sub.copy(&vehicle_odometry)) {
				memcpy(&_state_sharing.q, &vehicle_odometry.q, sizeof(_state_sharing.q));
				matrix::Eulerf euler{matrix::Quatf{vehicle_odometry.q}};
				_state_sharing.roll = euler(0);
				_state_sharing.pitch = euler(1);
				_state_sharing.yaw = euler(2);
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_state_sharing.local_position_x = vehicle_local_position.x;
				_state_sharing.local_position_y = vehicle_local_position.y;
				_state_sharing.local_position_z = vehicle_local_position.z;
			}
		}

		_predictions.updateData();
	}

	perf_end(_loop_perf);
}

PublisherStateSharing::PublisherStateSharing(StateSharing *parent, const px4::wq_config_t &config)
	: ScheduledWorkItem(kPublisherWorkItemName, config), _parent(parent)
{
}

void PublisherStateSharing::Run()
{
	if (!_parent) {
		return;
	}

	auto state_sharing = _parent->getStateSharing();
	state_sharing.timestamp = getRealTimeNs();
	state_sharing.timestamp_drone = hrt_absolute_time();

	if (_parent->shouldPublishOutgoingState()) {
		_outgoing_state_sharing_pub.publish(state_sharing);
	}

	_incoming_state_sharing_pub.publish(state_sharing);

	if (_parent->isFirstTimePublish()) {
		_parent->setFirstTimePublish(false);
		PX4_DEBUG("First state sharing published on [%f]",
			  getRealTimeNs() / 1e9);
	}
}

int StateSharing::task_spawn(int argc, char *argv[])
{
	StateSharing *instance = new StateSharing();

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

int StateSharing::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	_publisher_state_sharing.print_run_status();
	return 0;
}

int StateSharing::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int StateSharing::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description.
This module implements the logic to get data related to the state of the agent, and publish through uORB
in order to be shared with other modules or to other agents via MAVLink,
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("state_sharing", "state_sharing");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the execution of the module");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int state_sharing_main(int argc, char *argv[])
{
	return StateSharing::main(argc, argv);
}
