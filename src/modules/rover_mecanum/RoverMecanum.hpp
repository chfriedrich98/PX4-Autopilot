/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

// PX4 includes
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_mecanum_status.h>

// Standard libraries
#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>

// Local includes
#include "RoverMecanumGuidance/RoverMecanumGuidance.hpp"

using namespace time_literals;

class RoverMecanum : public ModuleBase<RoverMecanum>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RoverMecanum();
	~RoverMecanum() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/**
	 * @brief Turn normalized speed setpoints into normalized motor commands.
	 *
	 * @param forward_speed Normalized linear speed in body forward direction [-1, 1].
	 * @param lateral_speed Normalized linear speed in body lateral direction [-1, 1].
	 * @param speed_diff Normalized speed difference between left and right wheels [-1, 1].
	 * @return matrix::Vector4f: Normalized motor inputs [-1, 1]
	 */
	matrix::Vector4f computeMotorCommands(float forward_speed, float lateral_speed, float speed_diff);

protected:
	void updateParams() override;

private:
	void Run() override;

	/**
	 * @brief Update uORB subscriptions.
	 */
	void updateSubscriptions();

	// uORB Subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// uORB Publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<rover_mecanum_status_s> _rover_mecanum_status_pub{ORB_ID(rover_mecanum_status)};

	// Instances
	RoverMecanumGuidance _rover_mecanum_guidance{this};

	// Variables
	float _vehicle_body_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	int _nav_state{0};
	matrix::Quatf _vehicle_attitude_quaternion{};
	hrt_abstime _timestamp{0};
	PID_t _pid_yaw_rate; // The PID controller for yaw rate
	RoverMecanumGuidance::mecanum_setpoint _mecanum_setpoint;

	// Constants
	static constexpr float YAW_RATE_ERROR_THRESHOLD = 0.1f; // [rad/s] Error threshold for the closed loop yaw rate control

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_WHEEL_TRACK>) _param_rm_wheel_track,
		(ParamFloat<px4::params::RM_MAX_SPEED>) _param_rm_max_speed,
		(ParamFloat<px4::params::RM_MAN_YAW_SCALE>) _param_rm_man_yaw_scale,
		(ParamFloat<px4::params::RM_MAX_YAW_RATE>) _param_rm_max_yaw_rate,
		(ParamFloat<px4::params::RM_YAW_RATE_P>) _param_rm_yaw_rate_p,
		(ParamFloat<px4::params::RM_YAW_RATE_I>) _param_rm_yaw_rate_i,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
