/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf2_params.c
 * Parameter definition for ekf2.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

/**
 * Minimum time of arrival delta between non-IMU observations before data is downsampled.
 * Baro and Magnetometer data will be averaged before downsampling, other data will be point sampled resulting in loss of information.
 *
 * @group EKF2
 * @min 10
 * @max 50
 * @reboot_required true
 * @unit ms
 */
PARAM_DEFINE_INT32(EKF2_MIN_OBS_DT, 20);

/**
 * Magnetometer measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DELAY, 0);

/**
 * Barometer measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_DELAY, 0);

/**
 * GPS measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_DELAY, 110);

/**
 * Optical flow measurement delay relative to IMU measurements
 * Assumes measurement is timestamped at trailing edge of integration period
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_OF_DELAY, 5);

/**
 * Range finder measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_DELAY, 5);

/**
 * Airspeed measurement delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_ASP_DELAY, 100);

/**
 * Vision Position Estimator delay relative to IMU measurements
 *
 * @group EKF2
 * @min 0
 * @max 300
 * @unit ms
 * @reboot_required true
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_EV_DELAY, 175);

/**
 * Integer bitmask controlling GPS checks.
 *
 * Set bits to 1 to enable checks. Checks enabled by the following bit positions
 * 0 : Minimum required sat count set by EKF2_REQ_NSATS
 * 1 : Minimum required GDoP set by EKF2_REQ_GDOP
 * 2 : Maximum allowed horizontal position error set by EKF2_REQ_EPH
 * 3 : Maximum allowed vertical position error set by EKF2_REQ_EPV
 * 4 : Maximum allowed speed error set by EKF2_REQ_SACC
 * 5 : Maximum allowed horizontal position rate set by EKF2_REQ_HDRIFT. This check can only be used if the vehicle is stationary during alignment.
 * 6 : Maximum allowed vertical position rate set by EKF2_REQ_VDRIFT. This check can only be used if the vehicle is stationary during alignment.
 * 7 : Maximum allowed horizontal speed set by EKF2_REQ_HDRIFT. This check can only be used if the vehicle is stationary during alignment.
 * 8 : Maximum allowed vertical velocity discrepancy set by EKF2_REQ_VDRIFT
 *
 * @group EKF2
 * @min 0
 * @max 511
 * @bit 0 Min sat count (EKF2_REQ_NSATS)
 * @bit 1 Min GDoP (EKF2_REQ_GDOP)
 * @bit 2 Max horizontal position error (EKF2_REQ_EPH)
 * @bit 3 Max vertical position error (EKF2_REQ_EPV)
 * @bit 4 Max speed error (EKF2_REQ_SACC)
 * @bit 5 Max horizontal position rate (EKF2_REQ_HDRIFT)
 * @bit 6 Max vertical position rate (EKF2_REQ_VDRIFT)
 * @bit 7 Max horizontal speed (EKF2_REQ_HDRIFT)
 * @bit 8 Max vertical velocity discrepancy (EKF2_REQ_VDRIFT)
 */
PARAM_DEFINE_INT32(EKF2_GPS_CHECK, 21);

/**
 * Required EPH to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH, 5.0f);

/**
 * Required EPV to use GPS.
 *
 * @group EKF2
 * @min 2
 * @max 100
 * @unit m
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_EPV, 8.0f);

/**
 * Required speed accuracy to use GPS.
 *
 * @group EKF2
 * @min 0.5
 * @max 5.0
 * @unit m/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_SACC, 1.0f);

/**
 * Required satellite count to use GPS.
 *
 * @group EKF2
 * @min 4
 * @max 12
 */
PARAM_DEFINE_INT32(EKF2_REQ_NSATS, 6);

/**
 * Required GDoP to use GPS.
 *
 * @group EKF2
 * @min 1.5
 * @max 5.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_GDOP, 2.5f);

/**
 * Maximum horizontal drift speed to use GPS.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @unit m/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_HDRIFT, 0.3f);

/**
 * Maximum vertical drift speed to use GPS.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_REQ_VDRIFT, 0.5f);

/**
 * Rate gyro noise for covariance prediction.
 *
 * @group EKF2
 * @min 0.0001
 * @max 0.1
 * @unit rad/s
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_NOISE, 1.5e-2f);

/**
 * Accelerometer noise for covariance prediction.
 *
 * @group EKF2
 * @min 0.01
 * @max 1.0
 * @unit m/s/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_NOISE, 3.5e-1f);

/**
 * Process noise for IMU rate gyro bias prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 * @unit rad/s**2
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(EKF2_GYR_B_NOISE, 1.0e-3f);

/**
 * Process noise for IMU accelerometer bias prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.01
 * @unit m/s**3
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(EKF2_ACC_B_NOISE, 3.0e-3f);

/**
 * Process noise for body magnetic field prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.1
 * @unit Gauss/s
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_B_NOISE, 1.0e-4f);

/**
 * Process noise for earth magnetic field prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.1
 * @unit Gauss/s
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_E_NOISE, 1.0e-3f);

/**
 * Process noise for wind velocity prediction.
 *
 * @group EKF2
 * @min 0.0
 * @max 1.0
 * @unit m/s/s
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_WIND_NOISE, 1.0e-1f);

/**
 * Measurement noise for gps horizontal velocity.
 *
 * @group EKF2
 * @min 0.01
 * @max 5.0
 * @unit m/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_NOISE, 0.5f);

/**
 * Measurement noise for gps position.
 *
 * @group EKF2
 * @min 0.01
 * @max 10.0
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_NOISE, 0.5f);

/**
 * Measurement noise for non-aiding position hold.
 *
 * @group EKF2
 * @min 0.5
 * @max 50.0
 * @unit m
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_NOAID_NOISE, 10.0f);

/**
 * Measurement noise for barometric altitude.
 *
 * @group EKF2
 * @min 0.01
 * @max 15.0
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_NOISE, 2.0f);

/**
 * Measurement noise for magnetic heading fusion.
 *
 * @group EKF2
 * @min 0.01
 * @max 1.0
 * @unit rad
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_HEAD_NOISE, 0.3f);

/**
 * Measurement noise for magnetometer 3-axis fusion.
 *
 * @group EKF2
 * @min 0.001
 * @max 1.0
 * @unit Gauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_NOISE, 5.0e-2f);

/**
 * Measurement noise for airspeed fusion.
 *
 * @group EKF2
 * @min 0.5
 * @max 5.0
 * @unit m/s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_EAS_NOISE, 1.4f);

/**
 * Gate size for synthetic sideslip fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_BETA_GATE, 5.0f);

/**
 * Noise for synthetic sideslip fusion.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @unit m/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_BETA_NOISE, 0.3f);

/**
 * Magnetic declination
 *
 * @group EKF2
 * @unit deg
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_DECL, 0);

/**
 * Gate size for magnetic heading fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_HDG_GATE, 2.6f);

/**
 * Gate size for magnetometer XYZ component fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_GATE, 3.0f);

/**
 * Integer bitmask controlling handling of magnetic declination.
 *
 * Set bits in the following positions to enable functions.
 * 0 : Set to true to use the declination from the geo_lookup library when the GPS position becomes available, set to false to always use the EKF2_MAG_DECL value.
 * 1 : Set to true to save the EKF2_MAG_DECL parameter to the value returned by the EKF when the vehicle disarms.
 * 2 : Set to true to always use the declination as an observation when 3-axis magnetometer fusion is being used.
 *
 * @group EKF2
 * @min 0
 * @max 7
 * @bit 0 use geo_lookup declination
 * @bit 1 save EKF2_MAG_DECL on disarm
 * @bit 2 use declination as an observation
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_DECL_TYPE, 7);

/**
 * Type of magnetometer fusion
 *
 * Integer controlling the type of magnetometer fusion used - magnetic heading or 3-axis magnetometer.
 * If set to automatic: heading fusion on-ground and 3-axis fusion in-flight with fallback to heading fusion if there is insufficient motion to make yaw or mag biases observable.
 *
 * @group EKF2
 * @value 0 Automatic
 * @value 1 Magnetic heading
 * @value 2 3-axis fusion
 * @value 3 None
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_MAG_TYPE, 0);

/**
 * Horizontal acceleration threshold used by automatic selection of magnetometer fusion method.
 * This parameter is used when the magnetometer fusion method is set automatically (EKF2_MAG_TYPE = 0). If the filtered horizontal acceleration is greater than this parameter value, then the EKF will use 3-axis magnetomer fusion.
 *
 * @group EKF2
 * @min 0.0
 * @max 5.0
 * @unit m/s**2
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_ACCLIM, 0.5f);

/**
 * Yaw rate threshold used by automatic selection of magnetometer fusion method.
 * This parameter is used when the magnetometer fusion method is set automatically (EKF2_MAG_TYPE = 0). If the filtered yaw rate is greater than this parameter value, then the EKF will use 3-axis magnetomer fusion.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.5
 * @unit rad/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_MAG_YAWLIM, 0.25f);

/**
 * Gate size for barometric and GPS height fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_BARO_GATE, 5.0f);

/**
 * Gate size for GPS horizontal position fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE, 5.0f);

/**
 * Gate size for GPS velocity fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_V_GATE, 5.0f);

/**
 * Gate size for TAS fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_TAS_GATE, 3.0f);

/**
 * Integer bitmask controlling data fusion and aiding methods.
 *
 * Set bits in the following positions to enable:
 * 0 : Set to true to use GPS data if available
 * 1 : Set to true to use optical flow data if available
 * 2 : Set to true to inhibit IMU bias estimation
 * 3 : Set to true to enable vision position fusion
 * 4 : Set to true to enable vision yaw fusion
 * 5 : Set to true to enable multi-rotor drag specific force fusion
 * 6 : set to true if the EV observations are in a non NED reference frame and need to be rotated before being used
 *
 * @group EKF2
 * @min 0
 * @max 127
 * @bit 0 use GPS
 * @bit 1 use optical flow
 * @bit 2 inhibit IMU bias estimation
 * @bit 3 vision position fusion
 * @bit 4 vision yaw fusion
 * @bit 5 multi-rotor drag fusion
 * @bit 6 rotate external vision
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_AID_MASK, 1);

/**
 * Determines the primary source of height data used by the EKF.
 *
 * The range sensor option should only be used when for operation over a flat surface as the local NED origin will move up and down with ground level.
 *
 * @group EKF2
 * @value 0 Barometric pressure
 * @value 1 GPS
 * @value 2 Range sensor
 * @value 3 Vision
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_HGT_MODE, 1);

/**
 * Measurement noise for range finder fusion
 *
 * @group EKF2
 * @min 0.01
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_NOISE, 0.1f);

/**
 * Range finder range dependant noise scaler.
 *
 * Specifies the increase in range finder noise with range.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.2
 * @unit m/m
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_SFE, 0.05f);

/**
 * Gate size for range finder fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_GATE, 5.0f);

/**
 * Minimum valid range for the range finder
 *
 * @group EKF2
 * @min 0.01
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_MIN_RNG, 0.1f);


/**
 * Measurement noise for vision position observations used when the vision system does not supply error estimates
 *
 * @group EKF2
 * @min 0.01
 * @unit m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_EVP_NOISE, 0.05f);

/**
 * Measurement noise for vision angle observations used when the vision system does not supply error estimates
 *
 * @group EKF2
 * @min 0.01
 * @unit rad
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_EVA_NOISE, 0.05f);

/**
 * Gate size for vision estimate fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_EV_GATE, 5.0f);

/**
 * Measurement noise for the optical flow sensor when it's reported quality metric is at the maximum
 *
 * @group EKF2
 * @min 0.05
 * @unit rad/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MIN, 0.15f);

/**
 * Measurement noise for the optical flow sensor.
 *
 * (when it's reported quality metric is at the minimum set by EKF2_OF_QMIN).
 * The following condition must be met: EKF2_OF_N_MAXN >= EKF2_OF_N_MIN
 *
 * @group EKF2
 * @min 0.05
 * @unit rad/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_OF_N_MAX, 0.5f);

/**
 * Optical Flow data will only be used if the sensor reports a quality metric >= EKF2_OF_QMIN.
 *
 * @group EKF2
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(EKF2_OF_QMIN, 1);

/**
 * Gate size for optical flow fusion
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @group EKF2
 * @min 1.0
 * @unit SD
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_OF_GATE, 3.0f);

/**
 * Optical Flow data will not fused if the magnitude of the flow rate > EKF2_OF_RMAX
 *
 * @group EKF2
 * @min 1.0
 * @unit rad/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_OF_RMAX, 2.5f);

/**
 * Terrain altitude process noise - accounts for instability in vehicle height estimate
 *
 * @group EKF2
 * @min 0.5
 * @unit m/s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_NOISE, 5.0f);

/**
 * Magnitude of terrain gradient
 *
 * @group EKF2
 * @min 0.0
 * @unit m/m
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_TERR_GRAD, 0.5f);

/**
 * X position of IMU in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_X, 0.0f);

/**
 * Y position of IMU in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_Y, 0.0f);

/**
 * Z position of IMU in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_IMU_POS_Z, 0.0f);

/**
 * X position of GPS antenna in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_X, 0.0f);

/**
 * Y position of GPS antenna in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_Y, 0.0f);

/**
 * Z position of GPS antenna in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_GPS_POS_Z, 0.0f);

/**
 * X position of range finder origin in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_X, 0.0f);

/**
 * Y position of range finder origin in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_Y, 0.0f);

/**
 * Z position of range finder origin in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_POS_Z, 0.0f);

/**
 * X position of optical flow focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_X, 0.0f);

/**
 * Y position of optical flow focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_Y, 0.0f);

/**
 * Z position of optical flow focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_OF_POS_Z, 0.0f);

/**
* X position of VI sensor focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_X, 0.0f);

/**
 * Y position of VI sensor focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_Y, 0.0f);

/**
 * Z position of VI sensor focal point in body frame
 *
 * @group EKF2
 * @unit m
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_EV_POS_Z, 0.0f);

/**
* Airspeed fusion threshold. A value of zero will deactivate airspeed fusion. Any other positive
* value will determine the minimum airspeed which will still be fused.
*
* @group EKF2
* @min 0.0
* @unit m/s
* @decimal 1
*/
PARAM_DEFINE_FLOAT(EKF2_ARSP_THR, 0.0f);

/**
* Boolean determining if synthetic sideslip measurements should fused.
*
* A value of 1 indicates that fusion is active
*
* @group EKF2
* @boolean
*/
PARAM_DEFINE_INT32(EKF2_FUSE_BETA, 0);

/**

 * Time constant of the velocity output prediction and smoothing filter
 *
 * @group EKF2
 * @max 1.0
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_TAU_VEL, 0.25f);

/**
 * Time constant of the position output prediction and smoothing filter. Controls how tightly the output track the EKF states.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_TAU_POS, 0.25f);

/**
 * 1-sigma IMU gyro switch-on bias
 *
 * @group EKF2
 * @min 0.0
 * @max 0.2
 * @unit rad/sec
 * @reboot_required true
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_GBIAS_INIT, 0.1f);

/**
 * 1-sigma IMU accelerometer switch-on bias
 *
 * @group EKF2
 * @min 0.0
 * @max 0.5
 * @unit m/s/s
 * @reboot_required true
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_ABIAS_INIT, 0.2f);

/**
 * 1-sigma tilt angle uncertainty after gravity vector alignment
 *
 * @group EKF2
 * @min 0.0
 * @max 0.5
 * @unit rad
 * @reboot_required true
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_ANGERR_INIT, 0.1f);

/**
 * Range sensor pitch offset.
 *
 * @group EKF2
 * @min -0.75
 * @max 0.75
 * @unit rad
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_PITCH, 0.0f);

/**
 * Learned value of magnetometer X axis bias.
 * This is the amount of X-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_X, 0.0f);

/**
 * Learned value of magnetometer Y axis bias.
 * This is the amount of Y-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_Y, 0.0f);

/**
 * Learned value of magnetometer Z axis bias.
 * This is the amount of Z-axis magnetometer bias learned by the EKF and saved from the last flight. It must be set to zero if the ground based magnetometer calibration is repeated.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @reboot_required true
 * @unit mGauss
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_MAGBIAS_Z, 0.0f);

/**
 * ID of Magnetometer the learned bias is for.
 *
 * @group EKF2
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EKF2_MAGBIAS_ID, 0);

/**
 * State variance assumed for magnetometer bias storage.
 * This is a reference variance used to calculate the fraction of learned magnetometer bias that will be used to update the stored value. Smaller values will make the stored bias data adjust more slowly from flight to flight. Larger values will make it adjust faster.
 *
 * @group EKF2
 * @reboot_required true
 * @unit mGauss**2
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(EKF2_MAGB_VREF, 2.5E-7f);

/**
 * Maximum fraction of learned mag bias saved at each disarm.
 * Smaller values make the saved mag bias learn slower from flight to flight. Larger values make it learn faster. Must be > 0.0 and <= 1.0.
 *
 * @group EKF2
 * @min 0.0
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_MAGB_K, 0.2f);

/**
 * Range sensor aid.
 *
 * If this parameter is enabled then the estimator will make use of the range finder measurements
 * to estimate it's height even if range sensor is not the primary height source. It will only do so if conditions
 * for range measurement fusion are met.
 *
 * @group EKF2
 * @value 0 Range aid disabled
 * @value 1 Range aid enabled
 */
PARAM_DEFINE_INT32(EKF2_RNG_AID, 0);

/**
 * Maximum horizontal velocity allowed for range aid mode.
 *
 * If the vehicle horizontal speed exceeds this value then the estimator will not fuse range measurements
 * to estimate it's height. This only applies when range aid mode is activated (EKF2_RNG_AID = enabled).
 *
 * @group EKF2
 * @min 0.1
 * @max 2
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_VMAX, 1.0f);

/**
 * Maximum absolute altitude (height above ground level) allowed for range aid mode.
 *
 * If the vehicle absolute altitude exceeds this value then the estimator will not fuse range measurements
 * to estimate it's height. This only applies when range aid mode is activated (EKF2_RNG_AID = enabled).
 *
 * @group EKF2
 * @min 1.0
 * @max 10.0
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_HMAX, 5.0f);

/**
 * Gate size used for innovation consistency checks for range aid fusion
 *
 * A lower value means HAGL needs to be more stable in order to use range finder for height estimation
 * in range aid mode
 *
 * @group EKF2
 * @unit SD
 * @min 0.1
 * @max 5.0
 */
PARAM_DEFINE_FLOAT(EKF2_RNG_A_IGATE, 1.0f);

/**
 * Specific drag force observation noise variance used by the multi-rotor specific drag force model.
 * Increasing it makes the multi-rotor wind estimates adjust more slowly.
 *
 * @group EKF2
 * @min 0.5
 * @max 10.0
 * @unit (m/sec**2)**2
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_DRAG_NOISE, 2.5f);

/**
 * X-axis ballistic coefficient used by the multi-rotor specific drag force model.
 * This should be adjusted to minimise variance of the X-axis drag specific force innovation sequence.
 *
 * @group EKF2
 * @min 1.0
 * @max 100.0
 * @unit kg/m**2
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_BCOEF_X, 25.0f);

/**
 * Y-axis ballistic coefficient used by the multi-rotor specific drag force model.
 * This should be adjusted to minimise variance of the Y-axis drag specific force innovation sequence.
 *
 * @group EKF2
 * @min 1.0
 * @max 100.0
 * @unit kg/m**2
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_BCOEF_Y, 25.0f);

/**
 * Upper limit on airspeed along individual axes used to correct baro for position error effects
 *
 * @group EKF2
 * @min 5.0
 * @max 50.0
 * @unit m/s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_ASPD_MAX, 20.0f);

/**
 * Static pressure position error coefficient for the positive X axis
 * This is the ratio of static pressure error to dynamic pressure generated by a positive wind relative velocity along the X body axis.
 * If the baro height estimate rises during forward flight, then this will be a negative number.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_XP, 0.0f);

/**
 * Static pressure position error coefficient for the negative X axis.
 * This is the ratio of static pressure error to dynamic pressure generated by a negative wind relative velocity along the X body axis.
 * If the baro height estimate rises during backwards flight, then this will be a negative number.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_XN, 0.0f);

/**
 * Pressure position error coefficient for the Y axis.
 * This is the ratio of static pressure error to dynamic pressure generated by a wind relative velocity along the Y body axis.
 * If the baro height estimate rises during sideways flight, then this will be a negative number.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_Y, 0.0f);

/**
 * Static pressure position error coefficient for the Z axis.
 * This is the ratio of static pressure error to dynamic pressure generated by a wind relative velocity along the Z body axis.
 *
 * @group EKF2
 * @min -0.5
 * @max 0.5
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_PCOEF_Z, 0.0f);

/**
 * Accelerometer bias learning limit. The ekf delta velocity bias states will be limited to within a range equivalent to +- of this value.
 *
 * @group EKF2
 * @min 0.0
 * @max 0.8
 * @unit m/s/s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_LIM, 0.4f);

/**
 * Maximum IMU accel magnitude that allows IMU bias learning.
 * If the magnitude of the IMU accelerometer vector exceeds this value, the EKF delta velocity state estimation will be inhibited.
 * This reduces the adverse effect of high manoeuvre accelerations and IMU nonlinerity and scale factor errors on the delta velocity bias estimates.
 *
 * @group EKF2
 * @min 20.0
 * @max 200.0
 * @unit m/s/s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_ACCLIM, 25.0f);

/**
 * Maximum IMU gyro angular rate magnitude that allows IMU bias learning.
 * If the magnitude of the IMU angular rate vector exceeds this value, the EKF delta velocity state estimation will be inhibited.
 * This reduces the adverse effect of rapid rotation rates and associated errors on the delta velocity bias estimates.
 *
 * @group EKF2
 * @min 2.0
 * @max 20.0
 * @unit rad/s
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_GYRLIM, 3.0f);

/**
 * Time constant used by acceleration and angular rate magnitude checks used to inhibit delta velocity bias learning.
 * The vector magnitude of angular rate and acceleration used to check if learning should be inhibited has a peak hold filter applied to it with an exponential decay.
 * This parameter controls the time constant of the decay.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_ABL_TAU, 0.5f);
