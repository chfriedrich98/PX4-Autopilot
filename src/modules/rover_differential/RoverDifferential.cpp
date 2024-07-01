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

#include "RoverDifferential.hpp"
using namespace matrix;
using namespace time_literals;

RoverDifferential::RoverDifferential() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	pid_init(&_pid_angular_velocity, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_speed, PID_MODE_DERIVATIV_NONE, 0.001f);
	updateParams();
}

bool RoverDifferential::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverDifferential::updateParams()
{
	ModuleParams::updateParams();

	_wheel_base = _param_rd_wheel_base.get();
	_max_speed = _param_rd_wheel_speed.get() * _param_rd_wheel_radius.get();
	_rover_differential_guidance.setMaxSpeed(_max_speed);
	_max_angular_velocity = _max_speed / (_param_rd_wheel_base.get() / 2.f);
	_rover_differential_guidance.setMaxAngularVelocity(_max_angular_velocity);

	pid_set_parameters(&_pid_angular_velocity,
			   _param_rd_p_gain_angular_velocity.get(), // Proportional gain
			   _param_rd_i_gain_angular_velocity.get(), // Integral gain
			   0, // Derivative gain
			   20, // Integral limit
			   200); // Output limit

	pid_set_parameters(&_pid_speed,
			   _param_rd_p_gain_speed.get(), // Proportional gain
			   _param_rd_i_gain_speed.get(), // Integral gain
			   0, // Derivative gain
			   2, // Integral limit
			   200); // Output limit
}

void RoverDifferential::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	hrt_abstime now = hrt_absolute_time();
	const float dt = math::min((now - _time_stamp_last), 5000_ms) / 1e6f;
	_time_stamp_last = now;

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
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
	case vehicle_status_s::NAVIGATION_STATE_ACRO: {
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				_differential_setpoint.speed = manual_control_setpoint.throttle * math::max(0.f,
							       _param_rd_speed_scale.get()) * _max_speed;
				_differential_setpoint.yaw_rate = manual_control_setpoint.roll * _param_rd_ang_velocity_scale.get() *
								  _max_angular_velocity;
			}

			if (_nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {
				_differential_setpoint.closed_loop_speed_control = false;
				_differential_setpoint.closed_loop_yaw_rate_control = true;

			} else {
				_differential_setpoint.closed_loop_speed_control = false;
				_differential_setpoint.closed_loop_yaw_rate_control = false;
			}

		} break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_differential_setpoint = _rover_differential_guidance.computeGuidance(
						 _vehicle_yaw,
						 _vehicle_body_yaw_rate,
						 dt
					 );
		break;

	default: // Unimplemented nav states will stop the rover
		_differential_setpoint.speed = 0.f;
		_differential_setpoint.yaw_rate = 0.f;
		_differential_setpoint.closed_loop_speed_control = false;
		_differential_setpoint.closed_loop_yaw_rate_control = false;
		break;
	}

	// Closed loop setpoint control
	_differential_setpoint_control = _differential_setpoint;

	if (_differential_setpoint.closed_loop_speed_control) {
		_differential_setpoint_control.speed +=
			pid_calculate(&_pid_speed, _differential_setpoint.speed, _vehicle_forward_speed, 0, dt);
	}

	if (_differential_setpoint.closed_loop_yaw_rate_control) {
		_differential_setpoint_control.yaw_rate +=
			pid_calculate(&_pid_angular_velocity, _differential_setpoint.yaw_rate, _vehicle_body_yaw_rate, 0, dt);
	}

	// Calculate and publish motor commands
	Vector2f wheel_speeds =
		computeInverseKinematics(_differential_setpoint_control.speed, _differential_setpoint_control.yaw_rate);
	wheel_speeds = matrix::constrain(wheel_speeds, -1.f, 1.f);
	printf("Speed: %f Yaw rate: %f \n", (double)wheel_speeds(0), (double)wheel_speeds(1));
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	wheel_speeds.copyTo(actuator_motors.control);
	actuator_motors.timestamp = now;
	_actuator_motors_pub.publish(actuator_motors);

}

matrix::Vector2f RoverDifferential::computeInverseKinematics(float linear_velocity_x, float yaw_rate) const
{
	if (_max_speed < FLT_EPSILON) {
		return Vector2f();
	}

	linear_velocity_x = math::constrain(linear_velocity_x, -_max_speed, _max_speed);
	yaw_rate = math::constrain(yaw_rate, -_max_angular_velocity, _max_angular_velocity);

	const float rotational_velocity = (_wheel_base / 2.f) * yaw_rate;
	float combined_velocity = fabsf(linear_velocity_x) + fabsf(rotational_velocity);

	// Compute an initial gain
	float gain = 1.0f;

	if (combined_velocity > _max_speed) {
		float excess_velocity = fabsf(combined_velocity - _max_speed);
		const float adjusted_linear_velocity = fabsf(linear_velocity_x) - excess_velocity;
		gain = adjusted_linear_velocity / fabsf(linear_velocity_x);
	}

	// Apply the gain
	linear_velocity_x *= gain;

	// Calculate the left and right wheel speeds
	return Vector2f(linear_velocity_x - rotational_velocity,
			linear_velocity_x + rotational_velocity) / _max_speed;
}

int RoverDifferential::task_spawn(int argc, char *argv[])
{
	RoverDifferential *instance = new RoverDifferential();

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

int RoverDifferential::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverDifferential::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover Differential controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_differential", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_differential_main(int argc, char *argv[])
{
	return RoverDifferential::main(argc, argv);
}
