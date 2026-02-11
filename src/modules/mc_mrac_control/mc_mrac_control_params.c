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
 * INCIDENTAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_mrac_control_params.c
 * Parameters for MRAC-based multicopter rate and thrust control
 *
 * Channel mapping: 0=roll, 1=pitch, 2=yaw, 3=thrust
 * Parameter naming: MC_MRC_* (Multicopter MRaC)
 */

// ============================================================
// Reference Model Parameters
// ============================================================

/**
 * MRAC Roll Reference Model Coefficient A (Pole)
 *
 * Defines the roll rate reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_AM, 5.0f);

/**
 * MRAC Roll Reference Model Coefficient B (Gain)
 *
 * Defines the roll rate reference model input gain.
 * Typically bm = am for unit DC gain.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_BM, 5.0f);

/**
 * MRAC Pitch Reference Model Coefficient A (Pole)
 *
 * Defines the pitch rate reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_AM, 5.0f);

/**
 * MRAC Pitch Reference Model Coefficient B (Gain)
 *
 * Defines the pitch rate reference model input gain.
 * Typically bm = am for unit DC gain.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_BM, 5.0f);

/**
 * MRAC Yaw Reference Model Coefficient A (Pole)
 *
 * Defines the yaw rate reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response. Yaw typically uses slower response.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_AM, 3.0f);

/**
 * MRAC Yaw Reference Model Coefficient B (Gain)
 *
 * Defines the yaw rate reference model input gain.
 * Typically bm = am for unit DC gain.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_BM, 3.0f);

/**
 * MRAC Thrust Reference Model Coefficient A (Pole)
 *
 * Defines the thrust reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response. Uses default from PACC_MRAC_RM_AM.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_AM, 3.0f);

/**
 * MRAC Thrust Reference Model Coefficient B (Gain)
 *
 * Defines the thrust reference model input gain.
 * Typically bm = am for unit DC gain. Uses default from PACC_MRAC_RM_BM.
 *
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_BM, 3.0f);

// ============================================================
// Adaptation Gains
// ============================================================

/**
 * MRAC Roll Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the roll feedforward adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_GKR, 0.5f);

/**
 * MRAC Roll Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the roll feedback adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_GKX, 0.5f);

/**
 * MRAC Pitch Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the pitch feedforward adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_GKR, 0.5f);

/**
 * MRAC Pitch Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the pitch feedback adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_GKX, 0.5f);

/**
 * MRAC Yaw Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the yaw feedforward adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 * Yaw typically uses lower gains.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_GKR, 0.2f);

/**
 * MRAC Yaw Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the yaw feedback adaptive parameter.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_GKX, 0.2f);

/**
 * MRAC Thrust Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the thrust feedforward adaptive parameter.
 * Uses default from PACC_MRAC_G_KR.
 *
 * @min 0.001
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_GKR, 0.1f);

/**
 * MRAC Thrust Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the thrust feedback adaptive parameter.
 * Uses default from PACC_MRAC_G_KX.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_GKX, 0.1f);

// ============================================================
// Projection Bounds - Roll
// ============================================================

/**
 * MRAC Roll Kr Maximum Value
 *
 * Upper bound for roll feedforward adaptive parameter.
 * Prevents excessive feedforward action.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_KRMAX, 2.0f);

/**
 * MRAC Roll Kr Minimum Value
 *
 * Lower bound for roll feedforward adaptive parameter.
 * Prevents negative feedforward (reverse torque).
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_KRMIN, 0.0001f);

/**
 * MRAC Roll Kx Maximum Value
 *
 * Upper bound for roll feedback adaptive parameter.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_KXMAX, 1.0f);

/**
 * MRAC Roll Kx Minimum Value
 *
 * Lower bound for roll feedback adaptive parameter.
 * Allows negative feedback for damping.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_R_KXMIN, -2.0f);

// ============================================================
// Projection Bounds - Pitch
// ============================================================

/**
 * MRAC Pitch Kr Maximum Value
 *
 * Upper bound for pitch feedforward adaptive parameter.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_KRMAX, 2.0f);

/**
 * MRAC Pitch Kr Minimum Value
 *
 * Lower bound for pitch feedforward adaptive parameter.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_KRMIN, 0.0001f);

/**
 * MRAC Pitch Kx Maximum Value
 *
 * Upper bound for pitch feedback adaptive parameter.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_KXMAX, 1.0f);

/**
 * MRAC Pitch Kx Minimum Value
 *
 * Lower bound for pitch feedback adaptive parameter.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_P_KXMIN, -2.0f);

// ============================================================
// Projection Bounds - Yaw
// ============================================================

/**
 * MRAC Yaw Kr Maximum Value
 *
 * Upper bound for yaw feedforward adaptive parameter.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_KRMAX, 2.0f);

/**
 * MRAC Yaw Kr Minimum Value
 *
 * Lower bound for yaw feedforward adaptive parameter.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_KRMIN, 0.0001f);

/**
 * MRAC Yaw Kx Maximum Value
 *
 * Upper bound for yaw feedback adaptive parameter.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_KXMAX, 1.0f);

/**
 * MRAC Yaw Kx Minimum Value
 *
 * Lower bound for yaw feedback adaptive parameter.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_Y_KXMIN, -2.0f);

// ============================================================
// Projection Bounds - Thrust
// ============================================================

/**
 * MRAC Thrust Kr Maximum Value
 *
 * Upper bound for thrust feedforward adaptive parameter.
 * Uses default from PACC_MRAC_KR_MAX.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_KRMAX, 2.0f);

/**
 * MRAC Thrust Kr Minimum Value
 *
 * Lower bound for thrust feedforward adaptive parameter.
 * Uses default from PACC_MRAC_KR_MIN.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.00001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_KRMIN, 0.0001f);

/**
 * MRAC Thrust Kx Maximum Value
 *
 * Upper bound for thrust feedback adaptive parameter.
 * Uses default from PACC_MRAC_KX_MAX.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_KXMAX, 1.0f);

/**
 * MRAC Thrust Kx Minimum Value
 *
 * Lower bound for thrust feedback adaptive parameter.
 * Uses default from PACC_MRAC_KX_MIN.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_T_KXMIN, -2.0f);

// ============================================================
// Common Parameters
// ============================================================

/**
 * MRAC Low-Pass Filter Cutoff Frequency
 *
 * Cutoff frequency for filtering adaptive parameters.
 * Reduces high-frequency oscillations in control output.
 * Uses default from PACC_MRAC_LPF.
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @unit Hz
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_LPF_FRQ, 30.0f);

/**
 * MRAC Normalization Factor
 *
 * Normalization factor for adaptation law to prevent parameter divergence
 * with large input signals. Formula: 1 + norm * (r² + x²)
 * Uses default from PACC_MRAC_NORM.
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_NORM, 0.05f);

// ============================================================
// Module Selection Parameter
// ============================================================

/**
 * MRAC Control Mode
 *
 * Selects the rate control mode:
 * 0 = Standard PID control (mc_rate_control)
 * 1 = MRAC adaptive control (mc_mrac_control)
 *
 * When set to 1, both mc_rate_control and mc_mrac_control are started.
 * mc_rate_control handles manual mode, mc_mrac_control handles offboard mode.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @increment 1
 * @boolean
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_INT32(MC_MRC_MODE, 1);

// ============================================================
// Safety Parameters
// ============================================================

/**
 * MRAC Maximum Rate Limit
 *
 * Maximum angular rate limit for safety check.
 * If exceeded, the controller will reset.
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @unit rad/s
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_MAX_RT, 3.0f);

/**
 * MRAC Maximum Acceleration Limit
 *
 * Maximum proper acceleration limit for safety check.
 * If exceeded, the controller will reset.
 *
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @unit m/s^2
 * @group Multicopter MRAC Control
 */
PARAM_DEFINE_FLOAT(MC_MRC_MAX_AC, 5.0f);
