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

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/mrac_diagnostics.h>

using namespace time_literals;

/**
 * @brief MRAC-based multicopter rate and thrust control module
 *
 * This module implements Model Reference Adaptive Control (MRAC) for
 * full 4-DOF multicopter control:
 * - Channel 0: Roll rate -> Roll torque
 * - Channel 1: Pitch rate -> Pitch torque
 * - Channel 2: Yaw rate -> Yaw torque
 * - Channel 3: Proper acceleration -> Body-z thrust
 *
 * The module operates in all armed modes with rate control enabled.
 * This replaces mc_rate_control as the bottom-level rate controller.
 */
class MulticopterMracControl : public ModuleBase<MulticopterMracControl>,
                                public ModuleParams,
                                public px4::WorkItem {
 public:
  MulticopterMracControl();
  ~MulticopterMracControl() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  bool init();

 private:
  void Run() override;
  void parameters_updated();
  void resetFilters();

  /**
   * @brief Run MRAC algorithm for a single channel
   * @param channel Channel index (0=roll, 1=pitch, 2=yaw, 3=thrust)
   * @param r Reference setpoint
   * @param x Current measurement
   * @param dt Time step
   */
  void runSingleChannelMrac(int channel, float r, float x, float dt);

  /**
   * @brief Run MRAC for all 4 channels
   * @param dt Time step
   */
  void runMracChannels(float dt);

  /**
   * @brief Publish control outputs
   */
  void publishOutputs();

  /**
   * @brief Publish MRAC diagnostic data continuously (for logging)
   */
  void publishMracDiagnostics();

  /**
   * @brief Update actuator controls status for power monitoring
   */
  void updateActuatorControlsStatus(float dt);

  /**
   * @brief MRAC channel state structure
   *
   * Each control channel has its own MRAC state variables
   * for decentralized control.
   */
  struct MracChannel {
    // Reference model state (ideal trajectory)
    float ref_state{0.0f};

    // Adaptive parameter estimates
    float hat_kr{0.1f};  // Feedforward gain estimate
    float hat_kx{0.0f};  // Feedback gain estimate

    // Filtered adaptive parameters (for smooth control output)
    float hat_kr_filtered{0.1f};
    float hat_kx_filtered{0.0f};

    // Filtered measurement
    float measurement_filtered{0.0f};

    // Diagnostic data for publishing
    float setpoint{0.0f};      // Reference setpoint (r)
    float measurement{0.0f};   // Actual measurement (x)
    float tracking_error{0.0f}; // Tracking error (e = x - x_m)
  };

  // MRAC channels: [roll, pitch, yaw, thrust]
  static constexpr int NUM_CHANNELS = 4;
  MracChannel _mrac_channels[NUM_CHANNELS];

  // Control outputs: [torque_x, torque_y, torque_z, thrust_z]
  float _control_outputs[NUM_CHANNELS] {0.0f, 0.0f, 0.0f, 0.0f};

  // Subscriptions
  uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{
      this, ORB_ID(vehicle_angular_velocity)};
  uORB::SubscriptionData<vehicle_rates_setpoint_s>
      _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
  uORB::SubscriptionData<vehicle_local_position_setpoint_s>
      _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
  uORB::SubscriptionData<trajectory_setpoint_s>
      _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
  uORB::SubscriptionData<vehicle_acceleration_s>
      _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
  uORB::SubscriptionData<vehicle_attitude_s>
      _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
  uORB::SubscriptionData<vehicle_control_mode_s>
      _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
  uORB::SubscriptionData<vehicle_status_s>
      _vehicle_status_sub{ORB_ID(vehicle_status)};
  uORB::SubscriptionData<vehicle_land_detected_s>
      _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
  uORB::SubscriptionData<control_allocator_status_s>
      _control_allocator_status_sub{ORB_ID(control_allocator_status)};
  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

  // Publications
  uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{
      ORB_ID(vehicle_thrust_setpoint)};
  uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{
      ORB_ID(vehicle_torque_setpoint)};
  uORB::Publication<mrac_diagnostics_s> _mrac_diagnostics_pub{
      ORB_ID(mrac_diagnostics)};
  // Note: controller status telemetry can be added later with custom message
  uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{
      ORB_ID(actuator_controls_status_0)};
  orb_advert_t _mavlink_log_pub{nullptr};

  // State variables
  vehicle_angular_velocity_s _angular_velocity{};
  vehicle_acceleration_s _vehicle_acceleration{};
  vehicle_rates_setpoint_s _rates_setpoint{};
  vehicle_local_position_setpoint_s _local_pos_setpoint{};
  trajectory_setpoint_s _trajectory_setpoint{};
  vehicle_control_mode_s _vehicle_control_mode{};
  vehicle_status_s _vehicle_status{};
  vehicle_land_detected_s _vehicle_land_detected{};

  // Timing
  hrt_abstime _last_run{0};
  perf_counter_t _loop_perf{nullptr};

  // Control power monitoring
  float _energy_integration_time{0.0f};
  float _control_energy[NUM_CHANNELS] {};

  // Safety timeout
  hrt_abstime _last_setpoint_time{0};
  static constexpr hrt_abstime SETPOINT_TIMEOUT_us = 500000;  // 0.5 seconds

  // ============================================================
  // MRAC Parameters
  // ============================================================

  // Per-channel reference model parameters (am, bm)
  // Channel mapping: 0=roll, 1=pitch, 2=yaw, 3=thrust
  DEFINE_PARAMETERS(
      // === Reference Model Parameters ===
      (ParamFloat<px4::params::MC_MRC_R_AM>) _param_mrac_roll_am,
      (ParamFloat<px4::params::MC_MRC_R_BM>) _param_mrac_roll_bm,
      (ParamFloat<px4::params::MC_MRC_P_AM>) _param_mrac_pitch_am,
      (ParamFloat<px4::params::MC_MRC_P_BM>) _param_mrac_pitch_bm,
      (ParamFloat<px4::params::MC_MRC_Y_AM>) _param_mrac_yaw_am,
      (ParamFloat<px4::params::MC_MRC_Y_BM>) _param_mrac_yaw_bm,
      (ParamFloat<px4::params::MC_MRC_T_AM>) _param_mrac_thrust_am,
      (ParamFloat<px4::params::MC_MRC_T_BM>) _param_mrac_thrust_bm,

      // === Adaptation Gains ===
      (ParamFloat<px4::params::MC_MRC_R_GKR>) _param_mrac_roll_g_kr,
      (ParamFloat<px4::params::MC_MRC_R_GKX>) _param_mrac_roll_g_kx,
      (ParamFloat<px4::params::MC_MRC_P_GKR>) _param_mrac_pitch_g_kr,
      (ParamFloat<px4::params::MC_MRC_P_GKX>) _param_mrac_pitch_g_kx,
      (ParamFloat<px4::params::MC_MRC_Y_GKR>) _param_mrac_yaw_g_kr,
      (ParamFloat<px4::params::MC_MRC_Y_GKX>) _param_mrac_yaw_g_kx,
      (ParamFloat<px4::params::MC_MRC_T_GKR>) _param_mrac_thrust_g_kr,
      (ParamFloat<px4::params::MC_MRC_T_GKX>) _param_mrac_thrust_g_kx,

      // === Projection Bounds ===
      (ParamFloat<px4::params::MC_MRC_R_KRMAX>) _param_mrac_roll_kr_max,
      (ParamFloat<px4::params::MC_MRC_R_KRMIN>) _param_mrac_roll_kr_min,
      (ParamFloat<px4::params::MC_MRC_R_KXMAX>) _param_mrac_roll_kx_max,
      (ParamFloat<px4::params::MC_MRC_R_KXMIN>) _param_mrac_roll_kx_min,

      (ParamFloat<px4::params::MC_MRC_P_KRMAX>) _param_mrac_pitch_kr_max,
      (ParamFloat<px4::params::MC_MRC_P_KRMIN>) _param_mrac_pitch_kr_min,
      (ParamFloat<px4::params::MC_MRC_P_KXMAX>) _param_mrac_pitch_kx_max,
      (ParamFloat<px4::params::MC_MRC_P_KXMIN>) _param_mrac_pitch_kx_min,

      (ParamFloat<px4::params::MC_MRC_Y_KRMAX>) _param_mrac_yaw_kr_max,
      (ParamFloat<px4::params::MC_MRC_Y_KRMIN>) _param_mrac_yaw_kr_min,
      (ParamFloat<px4::params::MC_MRC_Y_KXMAX>) _param_mrac_yaw_kx_max,
      (ParamFloat<px4::params::MC_MRC_Y_KXMIN>) _param_mrac_yaw_kx_min,

      (ParamFloat<px4::params::MC_MRC_T_KRMAX>) _param_mrac_thrust_kr_max,
      (ParamFloat<px4::params::MC_MRC_T_KRMIN>) _param_mrac_thrust_kr_min,
      (ParamFloat<px4::params::MC_MRC_T_KXMAX>) _param_mrac_thrust_kx_max,
      (ParamFloat<px4::params::MC_MRC_T_KXMIN>) _param_mrac_thrust_kx_min,

      // === Common Parameters ===
      (ParamFloat<px4::params::MC_MRC_LPF_FRQ>) _param_mrac_lpf_cutoff,
      (ParamFloat<px4::params::MC_MRC_NORM>) _param_mrac_norm,

      // === Safety Parameters ===
      (ParamFloat<px4::params::MC_MRC_MAX_RT>) _param_mrac_max_rate,
      (ParamFloat<px4::params::MC_MRC_MAX_AC>) _param_mrac_max_acc,

      // === Thrust Conversion Parameters ===
      (ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover
  );

  // Parameter arrays for convenient access
  float _mrac_am[NUM_CHANNELS];
  float _mrac_bm[NUM_CHANNELS];
  float _mrac_gamma_kr[NUM_CHANNELS];
  float _mrac_gamma_kx[NUM_CHANNELS];
  float _mrac_kr_max[NUM_CHANNELS];
  float _mrac_kr_min[NUM_CHANNELS];
  float _mrac_kx_max[NUM_CHANNELS];
  float _mrac_kx_min[NUM_CHANNELS];

  // Low-pass filter time constant (updated from LPF_CUTOFF)
  float _rc{0.0f};
};
