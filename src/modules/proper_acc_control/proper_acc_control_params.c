/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file proper_acc_control_params.c
 * Parameters for proper acceleration control
 *
 * @author WarriorHanamy rongerch@mail2.sysu.edu.cn
 */

/**
 * proper acceleration error P gain
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 0.001
 * @max 0.008
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_P, 0.005f);


/**
 * proper acceleration setpoint low pass filter cutoff frequency 
 *
 * proper acceleration setpoint low pass filter cutoff frequency 
 *
 * @min 0.00
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_LPF_CUTOFF, 50.0f);

/**
 * proper acceleration curve over-confident SLOPE
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 20.0
 * @max 200.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_CUR_LIN_K, 150.0f);

/**
 * bound of delta proper acceleration on vehicle_angular_velocity coming
 *
 * bound of delta proper acceleration on vehicle_angular_velocity coming
 *
 * @min 0.0
 * @max 0.01
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_DELTA_BOUND, 0.005f);


/**
 * acc-thrust model blend
 *
 * u = (1-beta) * u + beta * model_ff
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_BETA, 0.0f);



/**
 * proper acceleration control timeout acc when mpc is lost
 *
 * proper acceleration control timeout acc when mpc is lost
 *
 * @min 9.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_TMO_ACC, 9.81f);


/**
 * proper acceleration control Max acc when mpc is running
 *
 * proper acceleration control Max acc when mpc is running
 *
 * @min 1.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_SFT_ACC, 3.0f);

/**
 * proper acceleration control Max rate when mpc is running
 *
 * proper acceleration control Max rate when mpc is running
 *
 * @min 1.5
 * @max 3.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PACC_SFT_RATE, 2.5f);


/**
 * proper acceleration control timeout time when mpc is lost  [0.1s]
 *
 * proper acceleration control timeout time when mpc is lost, for example,  5 is 0.5s.
 *
 * @min 1
 * @max 10
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(PACC_TMO_TIME, 5);



/**
 * proper acceleration control simulation mode
 *
 * proper acceleration control simulation mode, 1 is enable, 0 is disable.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(PACC_SIM, 1);


/**
 * Pitch torque linear gain K
 *
 * Pitch torque linear gain K
 *
 * @min 0.000
 * @max 0.3
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PITCH_TOR_K, 0.1f);

/**
 * Pitch torque Boundary
 *
 * Pitch torque Boundary
 *
 * @min 0.000
 * @max 0.080
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(PITCH_TOR_BD, 0.03f);

/**
 * MRAC Reference Model Coefficient A (Pole)
 *
 * Defines the reference model dynamics: ẋₘ = -am·xₘ + bm·r
 * Higher values = faster response. Typically am = bm for unit gain.
 *
 * @min 0.01
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_RM_AM, 3.0f);

/**
 * MRAC Reference Model Coefficient B (Gain)
 *
 * Defines the reference model input gain.
 * Typically bm = am for unit DC gain.
 *
 * @min 0.01
 * @max 20.0
 * @decimal 2
 * @increment 0.1
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_RM_BM, 3.0f);

/**
 * MRAC Adaptation Gain for Kr (Feedforward)
 *
 * Learning rate for the feedforward adaptive parameter Kr.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.00001
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_G_KR, 0.1f);

/**
 * MRAC Adaptation Gain for Kx (Feedback)
 *
 * Learning rate for the feedback adaptive parameter Kx.
 * Higher = faster adaptation but may cause oscillation.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.001
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_G_KX, 0.1f);

/**
 * MRAC Low-Pass Filter Cutoff Frequency
 *
 * Cutoff frequency for filtering adaptive parameters.
 * Reduces high-frequency oscillations in control output.
 *
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @unit Hz
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_LPF, 30.0f);

/**
 * MRAC Kr Maximum Value
 *
 * Upper bound for feedforward adaptive parameter.
 * Prevents excessive feedforward action.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_KR_MAX, 2.0f);

/**
 * MRAC Kr Minimum Value
 *
 * Lower bound for feedforward adaptive parameter.
 * Prevents negative feedforward (reverse thrust).
 *
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @increment 0.00001
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_KR_MIN, 0.0001f);

/**
 * MRAC Kx Maximum Value
 *
 * Upper bound for feedback adaptive parameter.
 *
 * @min 0.1
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_KX_MAX, 1.0f);

/**
 * MRAC Kx Minimum Value
 *
 * Lower bound for feedback adaptive parameter.
 * Allows negative feedback for damping.
 *
 * @min -5.0
 * @max -0.1
 * @decimal 3
 * @increment 0.01
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_KX_MIN, -2.0f);

/**
 * MRAC Normalization Factor
 *
 * Normalization factor for adaptation law to prevent parameter divergence
 * with large input signals. Formula: 1 + norm * (r² + x²)
 *
 * @min 0.001
 * @max 1.0
 * @decimal 4
 * @increment 0.001
 * @group Proper Acc Control
 */
PARAM_DEFINE_FLOAT(PACC_MRAC_NORM, 0.05f);
