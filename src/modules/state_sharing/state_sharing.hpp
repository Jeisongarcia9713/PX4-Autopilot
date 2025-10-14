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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/state_sharing_msg.h>
#include <uORB/topics/mission_command.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include "state_sharing/args_parser.hpp"
#include "state_sharing/utils.hpp"
#include "state_sharing/predictions.hpp"

class StateSharing;
class PublisherStateSharing : public px4::ScheduledWorkItem
{
	friend class StateSharing;
public:
	static constexpr const char *kPublisherWorkItemName = "publisher_state_sharing";

	PublisherStateSharing(StateSharing *parent, const px4::wq_config_t &config);

private:
	void Run() override;

	// Publications
	uORB::Publication<state_sharing_msg_s> _incoming_state_sharing_pub{ORB_ID(incoming_state_sharing)};
	uORB::Publication<state_sharing_msg_s> _outgoing_state_sharing_pub{ORB_ID(outgoing_state_sharing)};

	StateSharing *_parent{nullptr};
};

class StateSharing : public ModuleBase<StateSharing>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	StateSharing();
	~StateSharing() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static int create_publish_task(int argc, char *argv[]);

	/**
	 * @brief Initialize the state sharing module.
	 *
	 * @return true if initialization successful, false otherwise
	 */
	bool init();

	/**
	 * @brief Get the current state sharing message.
	 *
	 * @return Current state sharing message with position and attitude information
	 */
	state_sharing_msg_s getStateSharing();

	/**
	 * @brief Check if this is the first time publishing.
	 *
	 * @return true if this is the first publish, false otherwise
	 */
	bool isFirstTimePublish() const;

	/**
	 * @brief Set the first time publish flag.
	 *
	 * @param[in] first_time_publish Value to set the flag to
	 */
	void setFirstTimePublish(const bool &first_time_publish);

	/**
	 * @brief Check if outgoing state sharing should be published.
	 *
	 * @return true if outgoing state should be published, false otherwise
	 */
	bool shouldPublishOutgoingState() const;

	int print_status() override;

private:
	void Run() override;

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};        // subscription that schedules StateSharing when updated
	uORB::Subscription                 _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription                 _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionCallbackWorkItem _mission_command_sub{this, ORB_ID(incoming_mission_command)};
	uORB::Subscription                 _parameter_update_sub{ORB_ID(parameter_update)};


	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IDENT>) _param_ident,
		(ParamFloat<px4::params::SHARING_PERIOD>) _param_sharing_period,
		(ParamInt<px4::params::USE_PREDICTIONS>) _param_use_predictions
	)

	bool _start{false};
	bool _first_time_publish{false};
	state_sharing_msg_s _state_sharing{};
	Predictions _predictions;

	// Publisher helper
	PublisherStateSharing _publisher_state_sharing;

};
