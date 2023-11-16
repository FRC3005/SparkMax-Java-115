/*
 * Copyright (c) 2018-2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics;

import java.util.Optional;

public class CANPIDController {
  private final CANSparkMax m_device;

  public enum AccelStrategy {
    kTrapezoidal(0), kSCurve(1);

    @SuppressWarnings("MemberName")
    public final int value;

    AccelStrategy(int value) {
      this.value = value;
    }

    public static AccelStrategy fromInt(int value) {
        switch (value) {
            case 0:
                return kTrapezoidal;
            case 1:
                return kSCurve;
            default:
                return kTrapezoidal;
        }
    }
}

  /**
   * Constructs a CANPIDController.
   *
   * @param device The Spark Max this object configures.
   */
  public CANPIDController(CANSparkMax device) {
    m_device = device;
  }

  /**
   * Set the controller reference value based on the selected control mode.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl) {
    return setReference(value, ctrl, 0);
  }

  /**
   * Set the controller reference value based on the selected control mode.
   * This will override the pre-programmed control mode but not change what
   * is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type to override with
   *
   * @param pidSlot for this command
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl, int pidSlot) {
    return setReference(value, ctrl, pidSlot, 0);
  }

  /**
   * Set the controller reference value based on the selected control mode.
   * This will override the pre-programmed control mode but not change what
   * is programmed to the controller.
   *
   * @param value The value to set depending on the control mode. For basic
   * duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). Native units can be changed using the setPositionConversionFactor()
   * or setVelocityConversionFactor() methods of the CANEncoder class
   *
   * @param ctrl Is the control type to override with
   *
   * @param pidSlot for this command
   *
   * @param arbFeedforward A value from which is represented in voltage
   * applied to the motor after the result of the specified control mode. The
   * units for the parameter is Volts. This value is set after the control mode,
   * but before any current limits or ramp rates.
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setReference(double value, ControlType ctrl, int pidSlot,
                               double arbFeedforward) {
    return m_device.setpointCommand(value, ctrl, pidSlot, arbFeedforward);
  }

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The proportional gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setP(double gain) {
    return setP(gain, 0);
  }

  /**
   * Set the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The proportional gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setP(double gain, int slotID) {
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kP_0, slotID, gain);
  }

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The integral gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setI(double gain) {
    return setI(gain, 0);
  }

  /**
   * Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The integral gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setI(double gain, int slotID) {
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kI_0, slotID, gain);
  }

  /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The derivative gain value, must be positive
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setD(double gain) {
    return setD(gain, 0);
  }

    /**
   * Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The derivative gain value, must be positive
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setD(double gain, int slotID) {
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kD_0, slotID, gain);
  }

    /**
   * Set the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   * This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.
   *
   * @param gain The derivative filter value, must be a positive number between 0 and 1
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setDFilter(double gain, int slotID) {
    gain = gain < 0 ? 0 : gain;
    gain = gain > 1 ? 1 : gain;
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kDFilter_0, slotID, gain);
  }

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The feed-forward gain value
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setFF(double gain) {
    return setFF(gain, 0);
  }

  /**
   * Set the Feed-froward Gain constant of the PIDF controller on the SPARK
   * MAX. This uses the Set Parameter API and should be used infrequently. The
   * parameter does not presist unless burnFlash() is called.  The recommended
   * method to configure this parameter is use to SPARK MAX GUI to tune and
   * save parameters.
   *
   * @param gain The feed-forward gain value
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setFF(double gain, int slotID) {
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kF_0, slotID, gain);
  }

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value
   * specifies the range the |error| must be within for the integral constant
   * to take effect.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setIZone(double IZone) {
    return setIZone(IZone, 0);
  }

  /**
   * Set the IZone range of the PIDF controller on the SPARK MAX. This value
   * specifies the range the |error| must be within for the integral constant
   * to take effect.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param IZone The IZone value, must be positive. Set to 0 to disable
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setIZone(double IZone, int slotID) {
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kIZone_0, slotID,
        IZone);
  }

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   *
   * @param max Forward power maximum to allow the controller to output
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setOutputRange(double min, double max) {
    return setOutputRange(min, max, 0);
  }

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   * The recommended method to configure this parameter is to use the
   * SPARK MAX GUI to tune and save parameters.
   *
   * @param min Reverse power minimum to allow the controller to output
   *
   * @param max Forward power maximum to allow the controller to output
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to REV_OK if successful
   *
   */
  public CANError setOutputRange(double min, double max, int slotID) {
    if(m_device.getInverted()) {
      double tmp = min;
      min = max;
      max = tmp;
    }
    CANError status = setGainRaw(
        m_device, CANSparkMax.ConfigParameter.kOutputMin_0, slotID, min);
    if (status != CANError.kOK) {
        return status;
    }
    return setGainRaw(m_device, CANSparkMax.ConfigParameter.kOutputMax_0,
                      slotID, max);
  }

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double P Gain value
   *
   */
  public double getP() {
    return getP(0);
  }

  /**
   * Get the Proportional Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double P Gain value
   *
   */
  public double getP(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kP_0, slotID);
  }

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double I Gain value
   *
   */
  public double getI() {
    return getI(0);
  }

  /**
   * Get the Integral Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double I Gain value
   *
   */
  public double getI(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kI_0, slotID);
  }

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double D Gain value
   *
   */
  public double getD() {
    return getD(0);
  }

  /**
   * Get the Derivative Gain constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Gain value
   *
   */
  public double getD(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kD_0, slotID);
  }

    /**
   * Get the Derivative Filter constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Filter value
   *
   */
  public double getDFilter(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kDFilter_0, slotID);
  }

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double F Gain value
   *
   */
  public double getFF() {
    return getFF(0);
  }

  /**
   * Get the Feed-forward Gain constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double F Gain value
   *
   */
  public double getFF(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kF_0, slotID);
  }

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double IZone value
   *
   */
  public double getIZone() {
    return getIZone(0);
  }

  /**
   * Get the IZone constant of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double IZone value
   *
   */
  public double getIZone(int slotID) {
    return getGainRaw(m_device, CANSparkMax.ConfigParameter.kIZone_0, slotID);
  }

  /**
   * Get the derivative filter constant of the PIDF controller on the SPARK
   * MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double D Filter
   *
   */
  // public double getDFilter(int slotID = 0);

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double min value
   *
   */
  public double getOutputMin() {
    return getOutputMin(0);
  }

  /**
   * Get the min output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double min value
   *
   */
  public double getOutputMin(int slotID) {
    CANSparkMax.ConfigParameter param;
    if(m_device.getInverted()) {
      param = CANSparkMax.ConfigParameter.kOutputMax_0;
    } else {
      param = CANSparkMax.ConfigParameter.kOutputMin_0;
    }
    return getGainRaw(m_device, param, slotID);
  }

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @return double max value
   *
   */
  public double getOutputMax() {
    return getOutputMax(0);
  }

  /**
   * Get the max output of the PIDF controller on the SPARK MAX.
   *
   * This uses the Get Parameter API and should be used infrequently. This
   * function uses a non-blocking call and will return a cached value if the
   * parameter is not returned by the timeout. The timeout can be changed by
   * calling SetCANTimeout(int milliseconds)
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return double max value
   *
   */
  public double getOutputMax(int slotID) {
    CANSparkMax.ConfigParameter param;
    if(m_device.getInverted()) {
      param = CANSparkMax.ConfigParameter.kOutputMin_0;
    } else {
      param = CANSparkMax.ConfigParameter.kOutputMax_0;
    }
    return getGainRaw(m_device, param, slotID);
  }

  /**
   * Configure the maximum velocity of the SmartMotion mode. This is the
   * velocity that is reached in the middle of the profile and is what
   * the motor should spend most of its time at
   *
   * @param maxVel The maxmimum cruise velocity for the motion profile 
   * in RPM
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMaxVelocity(double maxVel, int slotID) {
    return setMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionMaxVelocity_0,
        slotID, maxVel);
  }

  /**
   * Configure the maximum acceleration of the SmartMotion mode. This is
   * the accleration that the motor velocity will increase at until the
   * max velocity is reached
   *
   * @param maxAccel The maxmimum acceleration for the motion profile 
   * in RPM per second
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMaxAccel(double maxAccel, int slotID) {
    return setMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionMaxAccel_0,
        slotID, maxAccel);
  }

  /**
   * Configure the mimimum velocity of the SmartMotion mode. Any 
   * requested velocities below this value will be set to 0.
   *
   * @param minVel The minimum velocity for the motion profile in RPM
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
    return setMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionMinVelOutput_0,
        slotID, minVel);
  }

  /**
   * Configure the allowed closed loop error of SmartMotion mode.
   * This value is how much deviation from your setpoint is 
   * tolerated and is useful in preventing oscillation around your 
   * setpoint.
   *
   * @param allowedErr The allowed deviation for your setpoint vs 
   * actual position in rotations
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
    return setMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionAllowedClosedLoopError_0,
        slotID, allowedErr);
  }

  /**
   * Coming soon. Configure the acceleration strategy used to control 
   * acceleration on the motor. The current strategy is trapezoidal 
   * motion profiling.
   *
   * @param accelStrategy The acceleration strategy to use for the 
   * automatically generated motion profile
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setSmartMotionAccelStrategy(AccelStrategy accelStrategy, int slotID) {
    return setMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionAccelStrategy_0, 
        slotID, (double)accelStrategy.value);
  }

  /**
   * Get the maximum velocity of the SmartMotion mode. This is the 
   * velocity that is reached in the middle of the profile and is 
   * what the motor should spend most of its time at
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The maxmimum cruise velocity for the motion profile in
   * RPM
   */
  public double getSmartMotionMaxVelocity(int slotID) {
    return getMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionMaxVelocity_0, 
        slotID);
  }

  /**
   * Get the maximum acceleration of the SmartMotion mode. This is 
   * the accleration that the motor velocity will increase at until
   * the max velocity is reached
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The maxmimum acceleration for the motion profile in 
   * RPM per second
   */
  public double getSmartMotionMaxAccel(int slotID) {
    return getMotionParamRaw(m_device,
        CANSparkMax.ConfigParameter.kSmartMotionMaxAccel_0,
        slotID);
  }

  /**
   * Get the mimimum velocity of the SmartMotion mode. Any requested
   * velocities below this value will be set to 0.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The minimum velocity for the motion profile in RPM
   */
  public double getSmartMotionMinOutputVelocity(int slotID) {
    return getMotionParamRaw(m_device,
        CANSparkMax.ConfigParameter.kSmartMotionMinVelOutput_0,
        slotID);
  }

  /**
   * Get the allowed closed loop error of SmartMotion mode. This 
   * value is how much deviation from your setpoint is tolerated
   * and is useful in preventing oscillation around your setpoint.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The allowed deviation for your setpoint vs actual 
   * position in rotations
   * 
   */
  public double getSmartMotionAllowedClosedLoopError(int slotID) {
    return getMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionAllowedClosedLoopError_0, 
        slotID);
  }

  /**
   * Get the acceleration strategy used to control acceleration on 
   * the motor.The current strategy is trapezoidal motion profiling.
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The acceleration strategy to use for the automatically 
   * generated motion profile.
   * 
   */
  public AccelStrategy getSmartMotionAccelStrategy(int slotID) {
    return AccelStrategy.fromInt((int)getMotionParamRaw(m_device, 
        CANSparkMax.ConfigParameter.kSmartMotionAccelStrategy_0, 
        slotID));
  }

  /**
   * Configure the maximum I accumulator of the PID controller.
   * This value is used to constrain the I accumulator to help 
   * manage integral wind-up
   *
   * @param iMaxAccum The max value to contrain the I accumulator to
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setIMaxAccum(double iMaxAccum, int slotID) {
    if (m_device.getInverted()) {
      iMaxAccum = iMaxAccum * -1;
    }
    return setPIDLimitParamsRaw(m_device,
        CANSparkMax.ConfigParameter.kIMaxAccum_0, 
        slotID, iMaxAccum);
  }

  /**
   * Get the maximum I accumulator of the PID controller. 
   * This value is used to constrain the I accumulator to help manage
   * integral wind-up
   *
   * @param slotID Is the gain schedule slot, the value is a number
   * between 0 and 3. Each slot has its own set of gain values and
   * can be changed in each control frame using SetReference().
   *
   * @return The max value to contrain the I accumulator to
   */
  public double getIMaxAccum(int slotID) {
    double val = getPIDLimitParamsRaw(m_device,
        CANSparkMax.ConfigParameter.kIMaxAccum_0, 
        slotID);

    if(m_device.getInverted()) {
      val = val * -1;
    }
    return val;
  }

  /**
   * Set the I accumulator of the PID controller. This is useful
   * when wishing to force a reset on the I accumulator of the 
   * PID controller. You can also preset values to see how it 
   * will respond to certain I characteristics
   *
   * @param iAccum The value to set the I accumulator to
   *
   * @return CANError Set to kOK if successful
   */
  public CANError setIAccum(double iAccum) {
    if (m_device.getInverted()) {
      iAccum = iAccum * -1;
    }
    return m_device.setIAccum(iAccum);
  }

  /**
   * Get the I accumulator of the PID controller. This is useful
   * when wishing to see what the I accumulator value is to help 
   * with PID tuning
   *
   * @return The value of the I accumulator
   */
  public double getIAccum() {
    return m_device.getPeriodicStatus2().iAccum;
  }

  private static CANError setGainRaw(CANSparkMax device,
                             CANSparkMax.ConfigParameter kParamSlot0, int slotID,
                             double value) {
      if (slotID < 0 || slotID > 3 ||
          kParamSlot0.value < CANSparkMax.ConfigParameter.kP_0.value ||
          kParamSlot0.value > CANSparkMax.ConfigParameter.kOutputMax_0.value) {
          return CANError.kError;
      }

      int offset = slotID * 8 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      CANSparkMax.ParameterStatus status = device.setParameter(param, value);
      if (status == CANSparkMax.ParameterStatus.kOK) {
          return CANError.kOK;
      }
      return CANError.kError;
  }

  private static CANError setMotionParamRaw(CANSparkMax device,
                            CANSparkMax.ConfigParameter kParamSlot0, int slotID,
                            double value) {
      if (slotID < 0 || slotID > 3 ||
        kParamSlot0.value < CANSparkMax.ConfigParameter.kSmartMotionMaxVelocity_0.value ||
        kParamSlot0.value > CANSparkMax.ConfigParameter.kSmartMotionAccelStrategy_0.value) {
        return CANError.kError;
      }

      int offset = slotID * 5 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      CANSparkMax.ParameterStatus status = device.setParameter(param, value);
      if (status == CANSparkMax.ParameterStatus.kOK) {
          return CANError.kOK;
      }
      return CANError.kError;
  }

  private static CANError setPIDLimitParamsRaw(CANSparkMax device,
                             CANSparkMax.ConfigParameter kParamSlot0, int slotID,
                             double value) {
      if (slotID < 0 || slotID > 3 ||
          kParamSlot0.value < CANSparkMax.ConfigParameter.kIMaxAccum_0.value ||
          kParamSlot0.value > CANSparkMax.ConfigParameter.kSlot3Placeholder3_0.value) {
          return CANError.kError;
      }

      int offset = slotID * 4 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      CANSparkMax.ParameterStatus status = device.setParameter(param, value);
      if (status == CANSparkMax.ParameterStatus.kOK) {
          return CANError.kOK;
      }
      return CANError.kError;
  }

  private static double getGainRaw(CANSparkMax device,
                           CANSparkMax.ConfigParameter kParamSlot0, int slotID) {
      Optional<Double> value;

      if (slotID < 0 || slotID > 3 ||
          kParamSlot0.value < CANSparkMax.ConfigParameter.kP_0.value ||
          kParamSlot0.value > CANSparkMax.ConfigParameter.kOutputMax_3.value) {
          return 0;
      }

      int offset = slotID * 8 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      value = device.getParameterDouble(param);

      if (value.isPresent()) {
        return value.get();
      } else {
        return 0;
      }
  }


  private static double getMotionParamRaw(CANSparkMax device,
                           CANSparkMax.ConfigParameter kParamSlot0, int slotID) {
      Optional<Double> value;

      if (slotID < 0 || slotID > 3 ||
          kParamSlot0.value < CANSparkMax.ConfigParameter.kSmartMotionMaxVelocity_0.value ||
          kParamSlot0.value > CANSparkMax.ConfigParameter.kSmartMotionAccelStrategy_0.value) {
          return 0;
      }

      int offset = slotID * 5 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      value = device.getParameterDouble(param);

      if (value.isPresent()) {
        return value.get();
      } else {
        return 0;
      }
  }

  private static double getPIDLimitParamsRaw(CANSparkMax device,
                           CANSparkMax.ConfigParameter kParamSlot0, int slotID) {
      Optional<Double> value;

      if (slotID < 0 || slotID > 3 ||
          kParamSlot0.value < CANSparkMax.ConfigParameter.kIMaxAccum_0.value ||
          kParamSlot0.value > CANSparkMax.ConfigParameter.kSlot3Placeholder3_0.value) {
          return 0;
      }

      int offset = slotID * 4 + kParamSlot0.value;
      CANSparkMax.ConfigParameter param =
          CANSparkMax.ConfigParameter.fromId(offset);

      value = device.getParameterDouble(param);

      if (value.isPresent()) {
        return value.get();
      } else {
        return 0;
      }
  }
}
