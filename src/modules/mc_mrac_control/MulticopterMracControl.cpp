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

#include "MulticopterMracControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

// Gravity constant in m/s² (used for proper acceleration calculation)
static constexpr float GRAVITY_MS2 = 9.80665f;

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterMracControl::MulticopterMracControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {
  _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
  parameters_updated();
  _actuator_controls_status_pub.advertise();
}

MulticopterMracControl::~MulticopterMracControl() {
  perf_free(_loop_perf);
}

bool MulticopterMracControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }
  resetFilters();
  return true;
}

void MulticopterMracControl::resetFilters() {
  // Reset filter time constant
  const float cutoff_freq = _param_mrac_lpf_cutoff.get();
  _rc = 1.0f / (2.0f * M_PI_F * cutoff_freq);

  // Reset MRAC channel states
  for (int i = 0; i < NUM_CHANNELS; i++) {
    _mrac_channels[i].ref_state = 0.0f;
    _mrac_channels[i].hat_kr = 0.1f;
    _mrac_channels[i].hat_kx = 0.0f;
    _mrac_channels[i].hat_kr_filtered = 0.1f;
    _mrac_channels[i].hat_kx_filtered = 0.0f;
    _mrac_channels[i].measurement_filtered = 0.0f;
  }
}

void MulticopterMracControl::parameters_updated() {
  // Update parameter arrays from individual parameters
  // Channel mapping: 0=roll, 1=pitch, 2=yaw, 3=thrust

  // Reference model parameters
  _mrac_am[0] = _param_mrac_roll_am.get();
  _mrac_am[1] = _param_mrac_pitch_am.get();
  _mrac_am[2] = _param_mrac_yaw_am.get();
  _mrac_am[3] = _param_mrac_thrust_am.get();

  _mrac_bm[0] = _param_mrac_roll_bm.get();
  _mrac_bm[1] = _param_mrac_pitch_bm.get();
  _mrac_bm[2] = _param_mrac_yaw_bm.get();
  _mrac_bm[3] = _param_mrac_thrust_bm.get();

  // Adaptation gains
  _mrac_gamma_kr[0] = _param_mrac_roll_g_kr.get();
  _mrac_gamma_kr[1] = _param_mrac_pitch_g_kr.get();
  _mrac_gamma_kr[2] = _param_mrac_yaw_g_kr.get();
  _mrac_gamma_kr[3] = _param_mrac_thrust_g_kr.get();

  _mrac_gamma_kx[0] = _param_mrac_roll_g_kx.get();
  _mrac_gamma_kx[1] = _param_mrac_pitch_g_kx.get();
  _mrac_gamma_kx[2] = _param_mrac_yaw_g_kx.get();
  _mrac_gamma_kx[3] = _param_mrac_thrust_g_kx.get();

  // Projection bounds
  _mrac_kr_max[0] = _param_mrac_roll_kr_max.get();
  _mrac_kr_min[0] = _param_mrac_roll_kr_min.get();
  _mrac_kx_max[0] = _param_mrac_roll_kx_max.get();
  _mrac_kx_min[0] = _param_mrac_roll_kx_min.get();

  _mrac_kr_max[1] = _param_mrac_pitch_kr_max.get();
  _mrac_kr_min[1] = _param_mrac_pitch_kr_min.get();
  _mrac_kx_max[1] = _param_mrac_pitch_kx_max.get();
  _mrac_kx_min[1] = _param_mrac_pitch_kx_min.get();

  _mrac_kr_max[2] = _param_mrac_yaw_kr_max.get();
  _mrac_kr_min[2] = _param_mrac_yaw_kr_min.get();
  _mrac_kx_max[2] = _param_mrac_yaw_kx_max.get();
  _mrac_kx_min[2] = _param_mrac_yaw_kx_min.get();

  _mrac_kr_max[3] = _param_mrac_thrust_kr_max.get();
  _mrac_kr_min[3] = _param_mrac_thrust_kr_min.get();
  _mrac_kx_max[3] = _param_mrac_thrust_kx_max.get();
  _mrac_kx_min[3] = _param_mrac_thrust_kx_min.get();

  // Update filter time constant
  const float cutoff_freq = _param_mrac_lpf_cutoff.get();
  _rc = 1.0f / (2.0f * M_PI_F * cutoff_freq);
}

void MulticopterMracControl::runSingleChannelMrac(int channel, float r, float x, float dt) {
  if (channel < 0 || channel >= NUM_CHANNELS) {
    return;
  }

  MracChannel &ch = _mrac_channels[channel];

  // Get channel-specific parameters
  const float am = _mrac_am[channel];
  const float bm = _mrac_bm[channel];
  const float gamma_kr = _mrac_gamma_kr[channel];
  const float gamma_kx = _mrac_gamma_kx[channel];
  const float kr_max = _mrac_kr_max[channel];
  const float kr_min = _mrac_kr_min[channel];
  const float kx_max = _mrac_kx_max[channel];
  const float kx_min = _mrac_kx_min[channel];
  const float norm_factor = _param_mrac_norm.get();

  // ============================================================
  // MRAC Algorithm Implementation
  // ============================================================

  // --- 1. Reference Model Evolution ---
  // Formula: x_m_dot = -am * x_m + bm * r
  float ref_dot = -am * ch.ref_state + bm * r;
  ch.ref_state += ref_dot * dt;

  // --- 2. Tracking Error Calculation ---
  // Formula: e = x - x_m (plant_state - reference_model_state)
  float error = x - ch.ref_state;
  ch.tracking_error = error;  // Store for diagnostic publishing

  // --- 3. Adaptation Law ---
  // With normalization to prevent divergence from large inputs
  float norm = 1.0f + norm_factor * (r * r + x * x);
  if (norm < 1.0f) norm = 1.0f;

  // Raw update rates (before projection)
  float raw_dot_kr = -gamma_kr * r * error / norm;
  float raw_dot_kx = -gamma_kx * x * error / norm;

  // --- 4. Projection Operator ---
  // Prevent parameter estimates from leaving feasible region
  float update_rate_kr = raw_dot_kr;
  float update_rate_kx = raw_dot_kx;

  // Kr projection logic
  if (ch.hat_kr >= kr_max && raw_dot_kr > 0.0f) {
    update_rate_kr = 0.0f;
  } else if (ch.hat_kr <= kr_min && raw_dot_kr < 0.0f) {
    update_rate_kr = 0.0f;
  }

  // Kx projection logic
  if (ch.hat_kx >= kx_max && raw_dot_kx > 0.0f) {
    update_rate_kx = 0.0f;
  } else if (ch.hat_kx <= kx_min && raw_dot_kx < 0.0f) {
    update_rate_kx = 0.0f;
  }

  // --- 5. Parameter Integration ---
  ch.hat_kr += update_rate_kr * dt;
  ch.hat_kx += update_rate_kx * dt;

  // --- 6. Safety Clamp ---
  ch.hat_kr = math::constrain(ch.hat_kr, kr_min, kr_max);
  ch.hat_kx = math::constrain(ch.hat_kx, kx_min, kx_max);

  // --- 7. Low-Pass Filter for Smooth Control ---
  float alpha = dt / (_rc + dt);

  // Filter adaptive parameters for smooth control output
  ch.hat_kr_filtered = ch.hat_kr_filtered + alpha * (ch.hat_kr - ch.hat_kr_filtered);
  ch.hat_kx_filtered = ch.hat_kx_filtered + alpha * (ch.hat_kx - ch.hat_kx_filtered);

  // Filter measurement for smooth control
  ch.measurement_filtered = ch.measurement_filtered + alpha * (x - ch.measurement_filtered);

  // --- 8. Control Output Calculation ---
  // u = Kr_filtered * r + Kx_filtered * x_filtered
  _control_outputs[channel] = ch.hat_kr_filtered * r + ch.hat_kx_filtered * ch.measurement_filtered;
}

void MulticopterMracControl::runMracChannels(float dt) {
  // Get current measurements
  const Vector3f rates{_angular_velocity.xyz};
  const Vector3f acc{_vehicle_acceleration.xyz};

  // Get setpoints from vehicle_rates_setpoint
  // vehicle_rates_setpoint contains: roll, pitch, yaw (rad/s) and thrust_body[3]
  float setpoints[NUM_CHANNELS];
  float measurements[NUM_CHANNELS];

  // Roll rate channel
  setpoints[0] = _rates_setpoint.roll;
  measurements[0] = rates(0);

  // Pitch rate channel
  setpoints[1] = _rates_setpoint.pitch;
  measurements[1] = rates(1);

  // Yaw rate channel
  setpoints[2] = _rates_setpoint.yaw;
  measurements[2] = rates(2);

  // Thrust channel (proper acceleration)
  // Read thrust from vehicle_rates_setpoint.thrust_body[2] (published by mc_att_control)
  // This works in ALL modes: Position, Altitude, Manual, Offboard
  //
  // vehicle_rates_setpoint.thrust_body[2] contains normalized thrust [-1, 0] (negative for NED frame)
  // We need to convert this to proper acceleration setpoint (m/s²)
  //
  // Conversion: thrust (normalized) → proper acceleration
  // Both setpoint and measurement should be in FLU frame (Z-up positive)
  // FLU frame: positive = up, negative = down

  float proper_acc_sp = 0.0f;
  const float thrust_normalized = -_rates_setpoint.thrust_body[2];  // Convert from NED [-1,0] to [0,1]

  // Get hover thrust parameter (typically 0.3 - 0.6)
  const float hover_thrust = _param_mpc_thr_hover.get();

  // Safety check for valid hover thrust
  if (hover_thrust > 0.01f && PX4_ISFINITE(thrust_normalized)) {
    // Convert normalized thrust to proper acceleration in FLU frame (Z-up positive)
    // At thrust = hover_thrust, proper_acc = 0 (hovering)
    // At thrust > hover_thrust, proper_acc > 0 (going up)
    // At thrust < hover_thrust, proper_acc < 0 (going down)
    proper_acc_sp = (1.0f - thrust_normalized / hover_thrust) * GRAVITY_MS2;
  } else {
    proper_acc_sp = 0.0f;  // Fallback to hover
  }

  setpoints[3] = proper_acc_sp;  // Proper acceleration (m/s²) in FLU frame

  // Measurement: vehicle_acceleration uses FLU frame (Z-up positive)
  // Use acc(2) directly without sign flip to match setpoint frame
  measurements[3] = acc(2);  // Proper acceleration (m/s²) in FLU frame

  // Safety checks for NaN/infinite values
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (!PX4_ISFINITE(setpoints[i])) {
      setpoints[i] = 0.0f;
    }
    if (!PX4_ISFINITE(measurements[i])) {
      measurements[i] = 0.0f;
    }
  }

  // Run MRAC for each channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    _mrac_channels[i].setpoint = setpoints[i];
    _mrac_channels[i].measurement = measurements[i];
    runSingleChannelMrac(i, setpoints[i], measurements[i], dt);
  }
}

void MulticopterMracControl::publishOutputs() {
  // Publish thrust setpoint
  vehicle_thrust_setpoint_s thrust_sp{};
  thrust_sp.timestamp = hrt_absolute_time();
  thrust_sp.timestamp_sample = _angular_velocity.timestamp_sample;

  // Thrust in body frame (only z-axis for multicopter)
  thrust_sp.xyz[0] = 0.0f;
  thrust_sp.xyz[1] = 0.0f;

  // Convert MRAC proper acceleration output (m/s²) in FLU frame to normalized thrust [0, 1]
  // FLU frame: proper_acc positive = up, negative = down
  // At hover (proper_acc = 0): thrust = hover_thrust
  // At proper_acc = +9.81 (up): thrust decreases (less needed), can go to 0
  // At proper_acc = -9.81 (down): thrust increases (more needed), can go to 1.0
  const float proper_acc_out = _control_outputs[3];  // MRAC output in m/s² (FLU frame)
  const float hover_thrust = _param_mpc_thr_hover.get();
  float thrust_normalized = (1.0f - proper_acc_out / GRAVITY_MS2) * hover_thrust;
  thrust_normalized = math::constrain(thrust_normalized, 0.0f, 1.0f);

  thrust_sp.xyz[2] = -thrust_normalized;  // Convert to NED frame (Z-down positive)

  _vehicle_thrust_setpoint_pub.publish(thrust_sp);

  // Publish torque setpoint
  vehicle_torque_setpoint_s torque_sp{};
  torque_sp.timestamp = hrt_absolute_time();
  torque_sp.timestamp_sample = _angular_velocity.timestamp_sample;

  // Torque setpoints from MRAC (constrained to valid range)
  torque_sp.xyz[0] = math::constrain(_control_outputs[0], -1.0f, 1.0f);  // Roll
  torque_sp.xyz[1] = math::constrain(_control_outputs[1], -1.0f, 1.0f);  // Pitch
  torque_sp.xyz[2] = math::constrain(_control_outputs[2], -1.0f, 1.0f);  // Yaw

  _vehicle_torque_setpoint_pub.publish(torque_sp);
}

void MulticopterMracControl::publishMracDiagnostics() {
  mrac_diagnostics_s diag{};
  diag.timestamp = hrt_absolute_time();

  // MRAC control status (4 channels)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    diag.ref_setpoint[i] = _mrac_channels[i].setpoint;
    diag.measurement[i] = _mrac_channels[i].measurement;
    diag.measurement_filtered[i] = _mrac_channels[i].measurement_filtered;
    diag.ref_model_state[i] = _mrac_channels[i].ref_state;
    diag.tracking_error[i] = _mrac_channels[i].tracking_error;
    diag.hat_kr[i] = _mrac_channels[i].hat_kr;
    diag.hat_kx[i] = _mrac_channels[i].hat_kx;
    diag.hat_kr_filtered[i] = _mrac_channels[i].hat_kr_filtered;
    diag.hat_kx_filtered[i] = _mrac_channels[i].hat_kx_filtered;
    diag.control_output[i] = _control_outputs[i];
  }

  _mrac_diagnostics_pub.publish(diag);

  // Debug: Print channel values every 1 second (400Hz * 1s = 400 iterations)
  static int debug_counter = 0;
  if (++debug_counter >= 400) {
    debug_counter = 0;
    PX4_INFO("[MRAC Ch0/roll] sp:%.3f meas:%.3f", (double)diag.ref_setpoint[0], (double)diag.measurement[0]);
    PX4_INFO("[MRAC Ch1/pitch] sp:%.3f meas:%.3f", (double)diag.ref_setpoint[1], (double)diag.measurement[1]);
    PX4_INFO("[MRAC Ch2/yaw] sp:%.3f meas:%.3f", (double)diag.ref_setpoint[2], (double)diag.measurement[2]);
    PX4_INFO("[MRAC Ch3/thrust] sp:%.3f meas:%.3f", (double)diag.ref_setpoint[3], (double)diag.measurement[3]);
  }
}

void MulticopterMracControl::updateActuatorControlsStatus(float dt) {
  // Accumulate control energy for power monitoring
  for (int i = 0; i < 3; i++) {  // Only for torque channels
    _control_energy[i] += _control_outputs[i] * _control_outputs[i] * dt;
  }

  _energy_integration_time += dt;

  // Publish status every 500ms
  if (_energy_integration_time > 500e-3f) {
    actuator_controls_status_s status;
    status.timestamp = _angular_velocity.timestamp;

    for (int i = 0; i < 3; i++) {
      status.control_power[i] = _control_energy[i] / _energy_integration_time;
      _control_energy[i] = 0.0f;
    }

    _actuator_controls_status_pub.publish(status);
    _energy_integration_time = 0.0f;
  }
}

void MulticopterMracControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Check if parameters have changed
  if (_parameter_update_sub.updated()) {
    // Clear update
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);

    updateParams();
    parameters_updated();
  }

  // Run controller on gyro changes
  if (_vehicle_angular_velocity_sub.update(&_angular_velocity)) {
    const hrt_abstime now = _angular_velocity.timestamp_sample;

    // Guard against too small (< 0.125ms) and too large (> 20ms) dt's
    const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
    _last_run = now;

    // Check for updates in other topics
    _vehicle_control_mode_sub.copy(&_vehicle_control_mode);
    _vehicle_status_sub.copy(&_vehicle_status);
    _vehicle_land_detected_sub.copy(&_vehicle_land_detected);
    _vehicle_attitude_sub.update();

    // Update setpoint subscription (for all modes, not just offboard)
    _vehicle_rates_setpoint_sub.copy(&_rates_setpoint);
    _vehicle_local_position_setpoint_sub.copy(&_local_pos_setpoint);
    _trajectory_setpoint_sub.copy(&_trajectory_setpoint);

    // Update acceleration measurement (needed for MRAC)
    _vehicle_acceleration_sub.copy(&_vehicle_acceleration);

    // Update control allocator status (for anti-windup/saturation handling)
    _control_allocator_status_sub.update();

    // Run MRAC in all armed modes with rate control enabled
    // This replaces mc_rate_control as the bottom-level rate controller
    if (_vehicle_control_mode.flag_armed &&
        _vehicle_control_mode.flag_control_rates_enabled &&
        _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

      // Run MRAC for all channels with current setpoints
      runMracChannels(dt);

      // Publish control outputs (thrust and torque)
      publishOutputs();

      // Update actuator controls status
      updateActuatorControlsStatus(dt);
    }

    // Always publish MRAC diagnostics for logging (even when not armed)
    publishMracDiagnostics();
  }

  perf_end(_loop_perf);
}

int MulticopterMracControl::task_spawn(int argc, char *argv[]) {
  MulticopterMracControl *instance = new MulticopterMracControl();

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

int MulticopterMracControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int MulticopterMracControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This module implements Model Reference Adaptive Control (MRAC) for multicopter
rate and thrust control. It provides full 4-DOF adaptive control:
- Roll rate -> Roll torque
- Pitch rate -> Pitch torque
- Yaw rate -> Yaw torque
- Proper acceleration -> Body-z thrust

The module operates in all armed modes with rate control enabled.
This replaces mc_rate_control as the bottom-level rate controller.

Control is performed using MRAC with independent adaptive parameters for each
channel (decentralized control architecture).

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("mc_mrac_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int mc_mrac_control_main(int argc, char *argv[]) {
  return MulticopterMracControl::main(argc, argv);
}
