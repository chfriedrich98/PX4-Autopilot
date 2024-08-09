/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "RoverMecanumGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverMecanumGuidance::RoverMecanumGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_mecanum_guidance_status_pub.advertise();
	pid_init(&_pid_heading, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_throttle, PID_MODE_DERIVATIV_NONE, 0.001f);

}

void RoverMecanumGuidance::updateParams()
{
	ModuleParams::updateParams();

	_max_yaw_rate = _param_rm_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	pid_set_parameters(&_pid_heading,
			   _param_rm_p_gain_heading.get(),  // Proportional gain
			   _param_rm_i_gain_heading.get(),  // Integral gain
			   0.f,  // Derivative gain
			   _max_yaw_rate,  // Integral limit
			   _max_yaw_rate);  // Output limit
	pid_set_parameters(&_pid_throttle,
			   _param_rm_p_gain_speed.get(), // Proportional gain
			   _param_rm_i_gain_speed.get(), // Integral gain
			   0.f, // Derivative gain
			   1.f, // Integral limit
			   1.f); // Output limit
}

RoverMecanumGuidance::mecanum_setpoint RoverMecanumGuidance::computeGuidance(const float yaw,
		const float actual_speed, const int nav_state)
{
	// Initializations
	bool mission_finished{false};
	float desired_speed{0.f};
	// float desired_yaw_rate{0.f};
	Vector2f desired_velocity(0.f, 0.f);
	// float lateral_throttle{0.f};
	// hrt_abstime timestamp_prev = _timestamp;
	// _timestamp = hrt_absolute_time();
	// const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// uORB subscriber updates
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(local_position.ref_lat, local_position.ref_lon, local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(local_position.x, local_position.y);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypoints();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		mission_finished = mission_result.finished;
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}

	desired_speed = _param_rm_miss_spd_def.get();

	const float distance_to_next_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _curr_wp(0),
					  _curr_wp(1));

	if (_theta < 2.61799f) {
		if (_param_rm_max_jerk.get() > FLT_EPSILON && _param_rm_max_accel.get() > FLT_EPSILON) {
			desired_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rm_max_jerk.get(),
					_param_rm_max_accel.get(), distance_to_next_wp, 0.0f);
			desired_speed = math::constrain(desired_speed, -_param_rm_max_speed.get(), _param_rm_max_speed.get());
		}

	} else {
		desired_speed = _param_rm_max_speed.get();
	}

	const float desired_heading = _pure_pursuit.calcDesiredHeading(_curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
				      desired_speed);

	const float heading_error = matrix::wrap_pi(desired_heading - yaw);



	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	    && distance_to_next_wp < _param_nav_acc_rad.get()) { // Return to launch
		mission_finished = true;
	}

	if (!mission_finished) {
		desired_velocity = desired_speed * Vector2f(cosf(heading_error), sinf(heading_error));
	}


	// // State machine
	// if (!mission_finished && distance_to_next_wp > _param_nav_acc_rad.get()) {
	// 	if (_currentState == GuidanceState::STOPPED) {
	// 		_currentState = GuidanceState::DRIVING;
	// 	}

	// 	if (_currentState == GuidanceState::DRIVING && fabsf(heading_error) > _param_rm_trans_drv_trn.get()) {
	// 		pid_reset_integral(&_pid_heading);
	// 		_currentState = GuidanceState::SPOT_TURNING;

	// 	} else if (_currentState == GuidanceState::SPOT_TURNING && fabsf(heading_error) < _param_rm_trans_trn_drv.get()) {
	// 		pid_reset_integral(&_pid_heading);
	// 		_currentState = GuidanceState::DRIVING;
	// 	}

	// } else { // Mission finished or delay command
	// 	_currentState = GuidanceState::STOPPED;
	// }

	// // Guidance logic
	// switch (_currentState) {
	// case GuidanceState::DRIVING:
	// case GuidanceState::SPOT_TURNING: {
	// 		desired_speed = _param_rm_miss_spd_def.get();

	// 		if (_param_rm_max_jerk.get() > FLT_EPSILON && _param_rm_max_accel.get() > FLT_EPSILON) {
	// 			desired_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rm_max_jerk.get(),
	// 					_param_rm_max_accel.get(), distance_to_next_wp, 0.0f);
	// 			desired_speed = math::constrain(desired_speed, -_param_rm_max_speed.get(), _param_rm_max_speed.get());
	// 		}

	// 		desired_velocity = desired_speed * Vector2f(cosf(desired_heading), sinf(desired_heading));
	// 		// lateral_throttle = pid_calculate(&_pid_heading, heading_error, 0.f, 0.f, dt);
	// 	} break;

	// // case GuidanceState::SPOT_TURNING:
	// // 	if (actual_speed < TURN_MAX_VELOCITY) { // Wait for the rover to stop
	// // 		desired_yaw_rate = pid_calculate(&_pid_heading, heading_error, 0.f, 0.f, dt); // Turn on the spot
	// // 	}

	// // 	break;

	// case GuidanceState::STOPPED:
	// default:
	// 	desired_speed = 0.f;
	// 	// desired_yaw_rate = 0.f;
	// 	break;

	// }

	// Closed loop speed control
	// float throttle{0.f};

	// if (fabsf(desired_speed) < FLT_EPSILON) {
	// 	pid_reset_integral(&_pid_throttle);

	// } else {
	// 	throttle = pid_calculate(&_pid_throttle, desired_speed, actual_speed, 0,
	// 				 dt);

	// 	if (_param_rm_max_speed.get() > FLT_EPSILON) { // Feed-forward term
	// 		throttle += math::interpolate<float>(desired_speed,
	// 						     0.f, _param_rm_max_speed.get(),
	// 						     0.f, 1.f);
	// 	}
	// }

	// Publish mecanum controller status (logging)
	_rover_mecanum_guidance_status.timestamp = _timestamp;
	_rover_mecanum_guidance_status.desired_speed = desired_speed;
	_rover_mecanum_guidance_status.pid_throttle_integral = _pid_throttle.integral;
	_rover_mecanum_guidance_status.lookahead_distance = _pure_pursuit.getLookaheadDistance();
	_rover_mecanum_guidance_status.pid_heading_integral = _pid_heading.integral;
	_rover_mecanum_guidance_status.heading_error_deg = M_RAD_TO_DEG_F * heading_error;
	_rover_mecanum_guidance_status.state_machine = (uint8_t) _currentState;
	_rover_mecanum_guidance_status_pub.publish(_rover_mecanum_guidance_status);


	// Return setpoints
	mecanum_setpoint mecanum_setpoint_temp;
	mecanum_setpoint_temp.forward_throttle = math::interpolate(desired_velocity(0), -_param_rm_max_speed.get(),
			_param_rm_max_speed.get(), -0.5f, 0.5f);
	mecanum_setpoint_temp.lateral_throttle = math::interpolate(desired_velocity(1), -_param_rm_max_speed.get(),
			_param_rm_max_speed.get(), -0.5f, 0.5f);
	mecanum_setpoint_temp.yaw_rate = 0.f;
	mecanum_setpoint_temp.closed_loop_yaw_rate = true;
	printf("Forward: %f, Lateral: %f \n", (double)mecanum_setpoint_temp.forward_throttle,
	       (double)mecanum_setpoint_temp.lateral_throttle);
	return mecanum_setpoint_temp;

}

void RoverMecanumGuidance::updateWaypoints()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		_curr_wp = Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	} else {
		_curr_wp = Vector2d(0, 0);
	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		_prev_wp = Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos;
	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		_next_wp = Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		_next_wp = _home_position;
	}

	// NED waypoint coordinates
	_curr_wp_ned = _global_ned_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_ned_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_ned = _global_ned_proj_ref.project(_next_wp(0), _next_wp(1));

	// Distances
	const Vector2f curr_to_next_wp_ned = _next_wp_ned - _curr_wp_ned;
	const Vector2f curr_to_prev_wp_ned = _prev_wp_ned - _curr_wp_ned;
	float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
	cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
	_theta = acosf(cosin);
}
