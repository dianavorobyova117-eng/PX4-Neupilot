/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "ProperAccControl.hpp"

#include <drivers/drv_hrt.h>
// #include <circuit_breaker/circuit_breaker.h>
#include <px4_platform_common/events.h>

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

ProperAccControl::ProperAccControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {
  parameters_updated();
}

ProperAccControl::~ProperAccControl() { perf_free(_loop_perf); }

bool ProperAccControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }
  resetFilters();
  return true;
}

void ProperAccControl::resetFilters() {
  // _lpf.initButterSysLowpass(0, 0, 10, 250);
  _proper_sp_lpf.set_cutoff_frequency(_param_imu_gyro_cutoff.get(),
                                      _param_acc_lpf_cutoff_frq.get());

  _proper_sp_lpf.reset(_u_prev);
  _rate_sft_filter.setParameters(1 / _param_imu_gyro_rate_max.get(), 0.5f);
  _acc_sft_filter.setParameters(1 / _param_imu_gyro_rate_max.get(), 0.5f);
}

void ProperAccControl::parameters_updated() {
  // rate control parameters
  // The controller gain K is used to convert the parallel (P + I/s + sD) form
  // to the ideal (K * [1 + 1/sTi + sTd]) form
  _delta_acc_bound = _param_delta_acc_bound.get();
  _timeout_acc = _param_acc_timeout_acc.get();
  _timeout_time = _param_sys_timeout_time.get() * 1e5;
  _is_sim = _param_acc_sim.get();

  _pitch_torque_k = _param_pitch_torque_k.get();
  _pitch_torque_bd = _param_pitch_torque_bd.get();

  _acc_limit = _param_acc_sft_acc.get();
  _rate_limit = _param_acc_sft_rate.get();

  _attitude_control.setProportionalGain(
      Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(),
               _param_mc_yaw_p.get()),
      _param_mc_yaw_weight.get());

  // angular rate limits
  using math::radians;
  _attitude_control.setRateLimit(
      Vector3f(radians(_param_mc_rollrate_max.get()),
               radians(_param_mc_pitchrate_max.get()),
               radians(_param_mc_yawrate_max.get())));
  resetFilters();
}

float ProperAccControl::get_u_inverse_model(float target_at) {
  // TODO check max and min
  return target_at / (float)(9.8) * (float)(0.62063);
}

void ProperAccControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Check if parameters have changed
  if (_parameter_update_sub.updated()) {
    // clear update
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);

    updateParams();
    parameters_updated();
  }

  // IMPORTANT
  _vehicle_proper_acc_setpoint_sub.update();
  _vehicle_thrust_setpoint_sub.update();
  _vehicle_attitude_sub.update();
  // float last_thrust_sp = 0.0;
  if (_vehicle_angular_velocity_sub.updated()) {
    _vehicle_control_mode_sub.update(&_vehicle_control_mode);
    // // must enable proper_acc_control to allow it control VehicleThrust
    _vehicle_proper_acc_setpoint_sub.update();
    _vacc_sub.update();
    _can_run_offboard = (_vehicle_control_mode.flag_control_offboard_enabled &&
                         _vehicle_control_mode.flag_control_rates_enabled);
    if (_last_can_run && _can_run_offboard) {
      _last_run = _vehicle_proper_acc_setpoint_sub.get().timestamp;
      _proper_acc_sp = _vehicle_proper_acc_setpoint_sub.get().proper_acc_sp;
      _rates_setpoint =
          matrix::Vector3f(_vehicle_proper_acc_setpoint_sub.get().rates_sp);
      _proper_model_ff = _vehicle_proper_acc_setpoint_sub.get().model_ff;
      if ((_safety_check) && !safeCheck()) {
        _safety_check = false;

        safeAttitudeHolder();
      }
      if ((_safety_check) && (_last_run > 0) &&
          hrt_elapsed_time(&_last_run) > _timeout_time) {
        PX4_WARN("MPC Timeout");
        _safety_check = false;
        safeAttitudeHolder();
      }
      _u_prev = -_vehicle_thrust_setpoint_sub.get().xyz[2];
      // change to FLU setting
      _a_curr = -_vacc_sub.get().xyz[2];

      // Calculate dt for MRAC integration
      _vehicle_angular_velocity_sub.copy(&_ang_vel);
      const hrt_abstime timestamp = _ang_vel.timestamp;
      float dt = (timestamp - _timestamp_prev) / 1e6f;
      _timestamp_prev = timestamp;

      // Guard against invalid dt (first run or large gap)
      if (dt < 0.0001f || dt > 0.1f) {
        dt = 1.0f / _param_imu_gyro_rate_max.get();
      }

        // ============================================================
        // MRAC (Model Reference Adaptive Control)
        // ============================================================

        // Temporary variables
        float error = 0.0f;
        float update_rate_kr = 0.0f;
        float update_rate_kx = 0.0f;
        float p_gain = 1.0f;

        // --- 1. Reference Model Evolution ---
        // Formula: x_m_dot = -am * x_m + bm * r
        float ref_model_dot = -_mrac_ref_model_am.get() * _mrac_ref_state +
                              _mrac_ref_model_bm.get() * _proper_acc_sp;
        _mrac_ref_state += ref_model_dot * dt;

        // --- 2. Tracking Error Calculation ---
        // Formula: e = x - x_m (plant_state - reference_model_state)
        error = _a_curr - _mrac_ref_state;

        // --- 3. Adaptation Law ---
        // With normalization to prevent divergence from large inputs
        float norm_factor = 1.0f + _mrac_norm.get() * _proper_acc_sp * _proper_acc_sp + _mrac_norm.get() * _a_curr * _a_curr;
        if (norm_factor < 1.0f) norm_factor = 1.0f;

        // Raw update rates (before projection)
        float raw_dot_kr = -_mrac_gamma_kr.get() * _proper_acc_sp * error * p_gain / norm_factor;
        float raw_dot_kx = -_mrac_gamma_kx.get() * _a_curr * error * p_gain / norm_factor;

        // --- 4. Projection Operator ---
        // Get parameter limits from params
        const float KR_MAX = _mrac_kr_max.get();
        const float KR_MIN = _mrac_kr_min.get();
        const float KX_MAX = _mrac_kx_max.get();
        const float KX_MIN = _mrac_kx_min.get();

        // Kr projection logic
        if (_mrac_hat_kr >= KR_MAX && raw_dot_kr > 0.0f) {
            update_rate_kr = 0.0f;
        } else if (_mrac_hat_kr <= KR_MIN && raw_dot_kr < 0.0f) {
            update_rate_kr = 0.0f;
        } else {
            update_rate_kr = raw_dot_kr;
        }

        // Kx projection logic
        if (_mrac_hat_kx >= KX_MAX && raw_dot_kx > 0.0f) {
            update_rate_kx = 0.0f;
        } else if (_mrac_hat_kx <= KX_MIN && raw_dot_kx < 0.0f) {
            update_rate_kx = 0.0f;
        } else {
            update_rate_kx = raw_dot_kx;
        }

        // --- 5. Parameter Integration ---
        _mrac_hat_kr += update_rate_kr * dt;
        _mrac_hat_kx += update_rate_kx * dt;

        // --- 6. Safety Clamp ---
        _mrac_hat_kr = math::constrain(_mrac_hat_kr, KR_MIN, KR_MAX);
        _mrac_hat_kx = math::constrain(_mrac_hat_kx, KX_MIN, KX_MAX);

        // --- 7. Low-Pass Filter for Smooth Control ---
        float cutoff_freq = _mrac_lpf_cutoff.get();
        float rc = 1.0f / (2.0f * M_PI_F * cutoff_freq);
        float alpha = dt / (rc + dt);

        // Filter adaptive parameters for smooth control output
        _mrac_hat_kr_filtered = _mrac_hat_kr_filtered + alpha * (_mrac_hat_kr - _mrac_hat_kr_filtered);
        _mrac_hat_kx_filtered = _mrac_hat_kx_filtered + alpha * (_mrac_hat_kx - _mrac_hat_kx_filtered);

        // Filter acceleration measurement for smooth control
        _a_curr_filtered = _a_curr_filtered + alpha * (_a_curr - _a_curr_filtered);

        // --- 8. Control Output Calculation ---
        // u = Kr_filtered * r + Kx_filtered * x
        _u = _mrac_hat_kr_filtered * _proper_acc_sp + _mrac_hat_kx_filtered * _a_curr_filtered;

        // --- 9. Publish MRAC Control Status in vehicle_rates_setpoint ---
      _u = math::constrain<float>(_u, 0.0, 1.0);
      _vehicle_rates_setpoint.thrust_body[2] = -_u;
      _vehicle_rates_setpoint.roll = _rates_setpoint(0);
      _vehicle_rates_setpoint.pitch = _rates_setpoint(1);
      _vehicle_rates_setpoint.yaw = _rates_setpoint(2);
      _vehicle_rates_setpoint.timestamp = hrt_absolute_time();

      // Note: MRAC telemetry not published (vehicle_rates_setpoint doesn't have MRAC fields)
      _vehicle_rates_setpoint_pub.publish(_vehicle_rates_setpoint);
    } else {
      _u_prev = -_vehicle_thrust_setpoint_sub.get().xyz[2];
      _u = _u_prev;
      _safety_check = true;
    }
    _last_can_run = _can_run_offboard;

    // use rates setpoint topic

    // mavlink_log_info(&_mavlink_log_pub, "%f",
    // (double)vehicle_rates_setpoint.roll);
  }

  perf_end(_loop_perf);
}

void ProperAccControl::safeAttitudeHolder() {
  _proper_acc_sp = _timeout_acc;
  // identity setpoint
  matrix::Quatf q_cur(_vehicle_attitude_sub.get().q);
  const float yaw = Eulerf(q_cur).psi();
  matrix::Quatf q_sp = Eulerf(0.0, 0.0, yaw);
  _attitude_control.setAttitudeSetpoint(q_sp, 0.0);
  _rates_setpoint = _attitude_control.update(q_cur);
}

bool ProperAccControl::safeCheck() {
  // position check
  // TODO map limit

  // TODO trajectory error too large

  // TODO rates too high
  vehicle_angular_velocity_s _vehicle_angular_velocity;
  _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
  matrix::Vector3f rates{_vehicle_angular_velocity.xyz};

  // TODO accleration is too high
  matrix::Vector3f acc{_vacc_sub.get().xyz[0], _vacc_sub.get().xyz[1],
                       _vacc_sub.get().xyz[2] + 9.81f};

  if (!_safety_check) {
    return false;
  }

  if (_acc_sft_filter.update(abs(acc(2))) > _acc_limit) {
    PX4_WARN("Acc Limit Exceeded");
    PX4_WARN("Acc Curr: %f", (double)(acc(2)));
    PX4_WARN("Acc Filtered: %f", (double)(_acc_sft_filter.getState()));
    _safety_check = false;
    return false;
  }
  if (_rate_sft_filter.update(rates.norm()) > _rate_limit) {
    PX4_WARN("Rate Limit Exceeded");
    PX4_WARN("Rate Curr: %f", (double)(rates.norm()));
    PX4_WARN("Rate Filtered: %f", (double)(_rate_sft_filter.getState()));
    _safety_check = false;
    return false;
  }

  return true;
}

int ProperAccControl::task_spawn(int argc, char *argv[]) {
  ProperAccControl *instance = new ProperAccControl();

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

int ProperAccControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int ProperAccControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements the proper acceleration control (actual acceleration minus gravitational acceleration).
It takes proper acceleration setpoints and vehicle angular rate setpoints as inputs.

The controller uses Model Reference Adaptive Control (MRAC) for proper acceleration tracking.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("proper_acc_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int proper_acc_control_main(int argc, char *argv[]) {
  return ProperAccControl::main(argc, argv);
}
