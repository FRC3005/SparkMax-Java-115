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

import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import java.util.Optional;

public class CANEncoder {
    private final CANSparkMax m_device;

    /**
     * Constructs a CANPIDController.
     *
     * @param device The Spark Max to which the encoder is attached.
     */
    public CANEncoder(CANSparkMax device) {
      m_device = device;
    }

    /**
     * Get the position of the motor. This returns the native units
     * of 'rotations' by default, and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @return Number of rotations of the motor
     *
     */
    public double getPosition() {
      return m_device.getPeriodicStatus2().sensorPosition;
    }

    /**
     * Get the velocity of the motor. This returns the native units
     * of 'RPM' by default, and can be changed by a scale factor
     * using setVelocityConversionFactor().
     *
     * @return Number the RPM of the motor
     *
     */
    public double getVelocity() {
      return m_device.getPeriodicStatus1().sensorVelocity;
    }


    /**
     * Set the position of the encoder.  By default the units
     * are 'rotations' and can be changed by a scale factor
     * using setPositionConversionFactor().
     *
     * @param position Number of rotations of the motor
     *
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setPosition(double position) {
      return m_device.setEncPosition(position);
    }

    /**
     * Set the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position.
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setPositionConversionFactor(double factor) {
      CANSparkMaxLowLevel.ParameterStatus status = m_device.setParameter(ConfigParameter.kPositionConversionFactor, (float) factor);
      if (status == CANSparkMaxLowLevel.ParameterStatus.kOK) {
          return CANError.kOK;
      } else {
          return CANError.kError;
      }
    }

    /**
     * Set the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @param factor The conversion factor to multiply the native units by
     *
     * @return CANError Set to CANError.kOk if successful
     */
    public CANError setVelocityConversionFactor(double factor) {
      CANSparkMaxLowLevel.ParameterStatus status = m_device.setParameter(ConfigParameter.kVelocityConversionFactor, (float) factor);
      if (status == CANSparkMaxLowLevel.ParameterStatus.kOK) {
          return CANError.kOK;
      } else {
          return CANError.kError;
      }
    }

    /**
     * Get the conversion factor for position of the encoder.
     * Multiplied by the native output units to give you position
     *
     * @return The conversion factor for position
     */
    public double getPositionConversionFactor() {
      Optional<Double> value;
      value = m_device.getParameterDouble(ConfigParameter.kPositionConversionFactor);
      if (value.isPresent()) {
          return value.get();
      } else {
          return 0.0;
      }
    }

    /**
     * Get the conversion factor for velocity of the encoder.
     * Multiplied by the native output units to give you velocity
     *
     * @return The conversion factor for velocity
     */
    public double getVelocityConversionFactor() {
      Optional<Double> value;
      value = m_device.getParameterDouble(ConfigParameter.kVelocityConversionFactor);
      if (value.isPresent()) {
          return value.get();
      } else {
          return 0.0;
      }
    }
}
