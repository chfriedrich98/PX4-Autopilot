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

#include "RoverMecanum.hpp"
using namespace matrix;
using namespace time_literals;

RoverMecanum::RoverMecanum() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
	_rover_mecanum_status_pub.advertise();
	// pid_init(&_pid_yaw_rate, PID_MODE_DERIVATIV_NONE, 0.001f);
}

bool RoverMecanum::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverMecanum::updateParams()
{
	ModuleParams::updateParams();

	// _max_yaw_rate = _param_rd_max_yaw_rate.get() * M_DEG_TO_RAD_F;

	// pid_set_parameters(&_pid_yaw_rate,
	// 		   _param_rd_p_gain_yaw_rate.get(), // Proportional gain
	// 		   _param_rd_i_gain_yaw_rate.get(), // Integral gain
	// 		   0.f, // Derivative gain
	// 		   1.f, // Integral limit
	// 		   1.f); // Output limit

}

void RoverMecanum::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	// const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// uORB subscriber updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);
		_nav_state = vehicle_status.nav_state;
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_body_yaw_rate = vehicle_angular_velocity.xyz[2];
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_forward_speed = velocity_in_body_frame(0);
	}

	// Navigation modes
	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				_mecanum_setpoint.forward_throttle = manual_control_setpoint.throttle;
				_mecanum_setpoint.lateral_throttle = manual_control_setpoint.roll;
				_mecanum_setpoint.yaw_rate = manual_control_setpoint.yaw * _param_rm_man_yaw_scale.get();

			}

			_mecanum_setpoint.closed_loop_yaw_rate = false;
		} break;

	// case vehicle_status_s::NAVIGATION_STATE_ACRO: {
	// 		manual_control_setpoint_s manual_control_setpoint{};

	// 		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
	// 			_mecanum_setpoint.forward_throttle = manual_control_setpoint.throttle;
	// 			_mecanum_setpoint.lateral_throttle = manual_control_setpoint.yaw;
	// 			_mecanum_setpoint.yaw_rate = math::interpolate<float>(manual_control_setpoint.roll,
	// 							  -1.f, 1.f,
	// 							  -_max_yaw_rate, _max_yaw_rate);
	// 		}

	// 		_mecanum_setpoint.closed_loop_yaw_rate = true;
	// 	} break;

	// case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	// case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	// 	_mecanum_setpoint = _rover_mecanum_guidance.computeGuidance(_vehicle_yaw, _vehicle_forward_speed,
	// 				 _nav_state);
	// 	break;

	default: // Unimplemented nav states will stop the rover
		_mecanum_setpoint.forward_throttle = 0.f;
		_mecanum_setpoint.lateral_throttle = 0.f;
		_mecanum_setpoint.closed_loop_yaw_rate = false;
		break;
	}

	// float speed_diff_normalized = _mecanum_setpoint.yaw_rate;

	// // Closed loop yaw rate control
	// if (_mecanum_setpoint.closed_loop_yaw_rate) {
	// 	if (fabsf(_mecanum_setpoint.yaw_rate - _vehicle_body_yaw_rate) < YAW_RATE_ERROR_THRESHOLD) {
	// 		speed_diff_normalized = 0.f;
	// 		pid_reset_integral(&_pid_yaw_rate);

	// 	} else {
	// 		const float speed_diff = _mecanum_setpoint.yaw_rate * _param_rd_wheel_track.get(); // Feedforward
	// 		speed_diff_normalized = math::interpolate<float>(speed_diff, -_param_rd_max_speed.get(),
	// 					_param_rd_max_speed.get(), -1.f, 1.f);
	// 		speed_diff_normalized = math::constrain(speed_diff_normalized +
	// 							pid_calculate(&_pid_yaw_rate, _mecanum_setpoint.yaw_rate, _vehicle_body_yaw_rate, 0, dt),
	// 							-1.f, 1.f); // Feedback
	// 	}

	// } else {
	// 	pid_reset_integral(&_pid_yaw_rate);
	// }

	// Publish rover mecanum status (logging)
	rover_mecanum_status_s rover_mecanum_status{};
	rover_mecanum_status.timestamp = _timestamp;
	rover_mecanum_status.actual_speed = _vehicle_forward_speed;
	rover_mecanum_status.desired_yaw_rate_deg_s = M_RAD_TO_DEG_F * _mecanum_setpoint.yaw_rate;
	rover_mecanum_status.actual_yaw_rate_deg_s = M_RAD_TO_DEG_F * _vehicle_body_yaw_rate;
	rover_mecanum_status.pid_yaw_rate_integral = _pid_yaw_rate.integral;
	_rover_mecanum_status_pub.publish(rover_mecanum_status);

	// Publish to motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeMotorCommands(_mecanum_setpoint.forward_throttle, _mecanum_setpoint.lateral_throttle,
			     _mecanum_setpoint.yaw_rate).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

}

matrix::Vector4f RoverMecanum::computeMotorCommands(float linear_velocity_x, float linear_velocity_y,
		const float yaw_rate)
{
	const float wb = _param_rm_wheel_base.get(); // TODO: This function should only use normalized values

	// Prioritize ratio between forward and lateral velocity
	float combined_velocity =  fabsf(linear_velocity_x) + fabsf(linear_velocity_y);

	if (combined_velocity > 1.f) {
		linear_velocity_x /= combined_velocity;
		linear_velocity_y /= combined_velocity;
		combined_velocity = 1.f;
	}

	// Prioritize yaw rate
	const float total_velocity = combined_velocity + fabsf(2 * yaw_rate * wb);

	if (total_velocity > 1.f) {
		const float excess_velocity = fabsf(total_velocity - 1.f);
		linear_velocity_x -= sign(linear_velocity_x) * 0.5f * excess_velocity;
		linear_velocity_y -= sign(linear_velocity_y) * 0.5f * excess_velocity;
	}

	// Define input vector and matrix data
	float input_data[3] = {linear_velocity_x, linear_velocity_y, yaw_rate};
	Matrix<float, 3, 1> input(input_data);
	float m_data[12] = {1, -1, -2.f * wb, 1, 1, 2.f * wb, 1, 1, -2.f * wb, 1, -1, 2.f * wb};
	Matrix<float, 4, 3> m(m_data);

	// Perform matrix-vector multiplication
	auto result = m * input; // result is Matrix<float, 4, 1>

	// Initialize Vector4f with the scaled results
	Vector4f motor_commands(result(0, 0), result(1, 0), result(2, 0), result(3, 0));

	return motor_commands;
}

int RoverMecanum::task_spawn(int argc, char *argv[])
{
	RoverMecanum *instance = new RoverMecanum();

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

int RoverMecanum::custom_command(int argc, char *argv[])
{
	return print_usage("unk_timestampn command");
}

int RoverMecanum::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Mecanum controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_mecanum", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_mecanum_main(int argc, char *argv[])
{
	return RoverMecanum::main(argc, argv);
}
