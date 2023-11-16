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

import edu.wpi.first.hal.CANData;
import java.util.Optional;

public class CANSparkMax extends CANSparkMaxLowLevel implements AutoCloseable {
	public enum SensorType {
		kNoSensor(0), kHallSensor(1), kEncoder(2);

		@SuppressWarnings("MemberName")
		public final int value;

		SensorType(int value) {
			this.value = value;
		}

		public static SensorType fromId(int id) {
			switch (id) {
			case 1:
				return kHallSensor;
			case 2:
				return kEncoder;
			default:
				return kNoSensor;
			}
		}
	}

	public enum IdleMode {
		kCoast(0), kBrake(1);

		@SuppressWarnings("MemberName")
		public final int value;

		IdleMode(int value) {
			this.value = value;
		}

		public static IdleMode fromId(int id) {
			if (id == 1) {
				return kBrake;
			}
			return kCoast;
		}
	}

	public enum InputMode {
		kPWM(0), kCAN(1);

		@SuppressWarnings("MemberName")
		public final int value;

		InputMode(int value) {
			this.value = value;
		}

		public static InputMode fromId(int id) {
			if (id == 1) {
				return kCAN;
			}
			return kPWM;
		}
	}

	public enum FaultID {
		kBrownout(0), kOvercurrent(1), kOvervoltage(2), kMotorFault(3), kSensorFault(4), kStall(5), kEEPROMCRC(6),
		kCANTX(7), kCANRX(8), kHasReset(9), kDRVFault(10), kOtherFault(11), kSoftLimitFwd(12), kSoftLimitRev(13),
		kHardLimitFwd(14), kHardLimitRev(15);

		@SuppressWarnings("MemberName")
		public final int value;

		FaultID(int value) {
			this.value = value;
		}
	}

	public static class ExternalFollower {
		private final int arbId;
		private final int configId;

		public static final ExternalFollower kFollowerDisabled = new ExternalFollower(0, 0);
		public static final ExternalFollower kFollowerSparkMax = new ExternalFollower(0x2051800, 26);
		public static final ExternalFollower kFollowerPhoenix = new ExternalFollower(0x2040080, 27);

		public ExternalFollower(int arbId, int configId) {
			this.arbId = arbId;
			this.configId = configId;
		}
	}

	// Only used for Get() or Set() api
	private double m_setpoint;

	/**
	 * Create a new SPARK MAX Controller
	 *
	 * @param deviceID The device ID.
	 * @param type     The motor type connected to the controller. Brushless motors
	 *                 must be connected to their matching color and the hall sensor
	 *                 plugged in. Brushed motors must be connected to the Red and
	 *                 Black terminals only.
	 */
	public CANSparkMax(int deviceID, MotorType type) {
		super(deviceID, type);
		// Initialize conversion factors to 1
		getEncoder().setPositionConversionFactor(1);
		getEncoder().setVelocityConversionFactor(1);
		clearFaults();
	}

	/**
	 * Closes the SPARK MAX Controller
	 */
	@Override
	public void close() {
	}

	/**** Speed Controller Interface ****/
	/**
	 * Common interface for setting the speed of a speed controller.
	 *
	 * @param speed The speed to set. Value should be between -1.0 and 1.0.
	 */
	@Override
	public void set(double speed) {
		// Only for 'get' api
		m_setpoint = speed;
		setpointCommand(speed, ControlType.kDutyCycle);
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() {
		return m_setpoint;
	}

	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	@Override
	public void setInverted(boolean isInverted) {
		m_inverted = isInverted;
	}

	/**
	 * Common interface for returning the inversion state of a speed controller.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		return m_inverted;
	}

	/**
	 * Common interface for disabling a motor.
	 */
	@Override
	public void disable() {
		set(0);
	}

	@Override
	public void stopMotor() {
		set(0);
	}

	@Override
	public void pidWrite(double output) {
		set(output);
	}

	/******* Extended Functions *******/
	/**
	 * @return An object for interfacing with the integrated encoder.
	 */
	public CANEncoder getEncoder() {
		return new CANEncoder(this);
	}

	/**
	 * @return An object for interfacing with the integrated PID controller.
	 */
	public CANPIDController getPIDController() {
		return new CANPIDController(this);
	}

	/**
	 * @return An object for interfacing with the integrated forward limit switch.
	 *
	 * @param polarity Whether the limit switch is normally open or normally closed.
	 */
	public CANDigitalInput getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
		return new CANDigitalInput(this, CANDigitalInput.LimitSwitch.kForward, polarity);
	}

	/**
	 * @return An object for interfacing with the integrated reverse limit switch.
	 *
	 * @param polarity Whether the limit switch is normally open or normally closed.
	 */
	public CANDigitalInput getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
		return new CANDigitalInput(this, CANDigitalInput.LimitSwitch.kReverse, polarity);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 *
	 */
	public CANError setSmartCurrentLimit(int limit) {
		return setSmartCurrentLimit(limit, 0, 20000);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setSmartCurrentLimit(int stallLimit, int freeLimit) {
		return setSmartCurrentLimit(stallLimit, freeLimit, 20000);
	}

	/**
	 * Sets the current limit in Amps.
	 *
	 * The motor controller will reduce the controller voltage output to avoid
	 * surpassing this limit. This limit is enabled by default and used for
	 * brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 *
	 * The NEO Brushless Motor has a low internal resistance, which can mean large
	 * current spikes that could be enough to cause damage to the motor and
	 * controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 *
	 * The controller can also limit the current based on the RPM of the motor in a
	 * linear fashion to help with controllability in closed loop control. For a
	 * response that is linear the entire RPM range leave limit RPM at 0.
	 *
	 * @param stallLimit The current limit in Amps at 0 RPM.
	 * @param freeLimit  The current limit at free speed (5700RPM for NEO).
	 * @param limitRPM   RPM less than this value will be set to the stallLimit, RPM
	 *                   values greater than limitRPM will scale linearly to
	 *                   freeLimit
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
		ParameterStatus status = setParameter(ConfigParameter.kSmartCurrentStallLimit, stallLimit);
		if (status != ParameterStatus.kOK) {
			return CANError.kError;
		}

		status = setParameter(ConfigParameter.kSmartCurrentFreeLimit, freeLimit);
		if (status != ParameterStatus.kOK) {
			return CANError.kError;
		}

		status = setParameter(ConfigParameter.kSmartCurrentConfig, limitRPM);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit The current limit in Amps.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setSecondaryCurrentLimit(double limit) {
		return setSecondaryCurrentLimit(limit, 0);
	}

	/**
	 * Sets the secondary current limit in Amps.
	 *
	 * The motor controller will disable the output of the controller briefly if the
	 * current limit is exceeded to reduce the current. This limit is a simplified
	 * 'on/off' controller. This limit is enabled by default but is set higher than
	 * the default Smart Current Limit.
	 *
	 * The time the controller is off after the current limit is reached is
	 * determined by the parameter limitCycles, which is the number of PWM cycles
	 * (20kHz). The recommended value is the default of 0 which is the minimum time
	 * and is part of a PWM cycle from when the over current is detected. This
	 * allows the controller to regulate the current close to the limit value.
	 *
	 * The total time is set by the equation
	 *
	 * <code>
	 * t = (50us - t0) + 50us * limitCycles
	 * t = total off time after over current
	 * t0 = time from the start of the PWM cycle until over current is detected
	 * </code>
	 *
	 *
	 * @param limit      The current limit in Amps.
	 * @param chopCycles The number of additional PWM cycles to turn the driver off
	 *                   after overcurrent is detected.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setSecondaryCurrentLimit(double limit, int chopCycles) {
		ParameterStatus status = setParameter(ConfigParameter.kCurrentChop, limit);
		if (status != ParameterStatus.kOK) {
			return CANError.kError;
		}

		status = setParameter(ConfigParameter.kCurrentChopCycles, chopCycles);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Sets the idle mode setting for the SPARK MAX.
	 *
	 * @param mode Idle mode (coast or brake).
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setIdleMode(IdleMode mode) {
		ParameterStatus status = setParameter(ConfigParameter.kIdleMode, mode.value);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Gets the idle mode setting for the SPARK MAX.
	 *
	 * This uses the Get Parameter API and should be used infrequently. This
	 * function uses a non-blocking call and will return a cached value if the
	 * parameter is not returned by the timeout. The timeout can be changed by
	 * calling SetCANTimeout(int milliseconds)
	 *
	 * @return IdleMode Idle mode setting
	 */
	public IdleMode getIdleMode() {
		Optional<Integer> value = getParameterInt(ConfigParameter.kIdleMode);
		if (value.isPresent()) {
			return IdleMode.fromId(value.get());
		} else {
			return IdleMode.kBrake;
		}
	}

	/**
	 * Sets the voltage compensation setting for all modes on the SPARK MAX and
	 * enables voltage compensation.
	 *
	 * @param nominalVoltage Nominal voltage to compensate output to
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError enableVoltageCompensation(double nominalVoltage) {
		boolean statusOkay = setParameter(ConfigParameter.kCompensatedNominalVoltage,
				nominalVoltage) == ParameterStatus.kOK;
		statusOkay &= setParameter(ConfigParameter.kVoltageCompMode, 2) == ParameterStatus.kOK;
		if (statusOkay) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Disables the voltage compensation setting for all modes on the SPARK MAX.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError disableVoltageCompensation() {
		boolean statusOkay = setParameter(ConfigParameter.kVoltageCompMode, 0) == ParameterStatus.kOK;
		statusOkay &= setParameter(ConfigParameter.kCompensatedNominalVoltage, (double) 0) == ParameterStatus.kOK;
		if (statusOkay) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Get the configured voltage compensation nominal voltage value
	 *
	 * @return The nominal voltage for voltage compensation mode.
	 */
	public double getVoltageCompensationNominalVoltage() {
		Optional<Double> value;
		value = getParameterDouble(ConfigParameter.kCompensatedNominalVoltage);
		if (value.isPresent()) {
			return value.get();
		} else {
			return 0.0;
		}
	}

	/**
	 * Sets the ramp rate for open loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setOpenLoopRampRate(double rate) {
		if (rate != 0) {
			rate = 1.0 / rate;
		}
		ParameterStatus status = setParameter(ConfigParameter.kOpenLoopRampRate, (float) rate);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Sets the ramp rate for closed loop control modes.
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @param rate Time in seconds to go from 0 to full throttle.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setClosedLoopRampRate(double rate) {
		if (rate != 0) {
			rate = 1.0 / rate;
		}
		ParameterStatus status = setParameter(ConfigParameter.kClosedLoopRampRate, (float) rate);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Get the configured open loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getOpenLoopRampRate() {
		Optional<Double> value;
		value = getParameterDouble(ConfigParameter.kOpenLoopRampRate);
		if (value.isPresent()) {
			// Convert to 'time from 0 to full throttle'
			if (value.get() != 0) {
				return 1.0 / value.get();
			}
			return value.get();
		} else {
			return 0.0;
		}
	}

	/**
	 * Get the configured closed loop ramp rate
	 *
	 * This is the maximum rate at which the motor controller's output is allowed to
	 * change.
	 *
	 * @return ramp rate time in seconds to go from 0 to full throttle.
	 */
	public double getClosedLoopRampRate() {
		Optional<Double> value;
		value = getParameterDouble(ConfigParameter.kClosedLoopRampRate);
		if (value.isPresent()) {
			// Convert to 'time from 0 to full throttle'
			if (value.get() != 0) {
				return 1.0 / value.get();
			}
			return value.get();
		} else {
			return 0.0;
		}
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 *
	 * @param leader The motor controller to follow.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError follow(final CANSparkMax leader) {
		return follow(leader, false);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 *
	 * @param leader The motor controller to follow.
	 * @param invert Set the follower to output opposite of the leader
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError follow(final CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSparkMax, leader.getDeviceId(), invert);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError follow(ExternalFollower leader, int deviceID) {
		return follow(leader, deviceID, m_inverted);
	}

	/**
	 * Causes this controller's output to mirror the provided leader.
	 *
	 * Only voltage output is mirrored. Settings changed on the leader do not affect
	 * the follower.
	 *
	 * @param leader   The type of motor controller to follow (Talon SRX, Spark Max,
	 *                 etc.).
	 * @param deviceID The CAN ID of the device to follow.
	 *
	 * @param invert   Set the follower to output opposite of the leader
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError follow(ExternalFollower leader, int deviceID, boolean invert) {
		FollowConfig maxFollower = new FollowConfig();
		maxFollower.leaderArbId = leader.arbId | deviceID;
		maxFollower.config.predefined = leader.configId;
		maxFollower.config.invert = invert ? 1 : 0;
		return setFollow(maxFollower);
	}

	/**
	 * Returns whether the controller is following another controller
	 *
	 * @return True if this device is following another controller false otherwise
	 */
	public boolean isFollower() {
		return getPeriodicStatus0().isFollower;
	}

	/**
	 * @return All fault bits as a short
	 */
	public short getFaults() {
		return getPeriodicStatus0().faults;
	}

	/**
	 * @return All sticky fault bits as a short
	 */
	public short getStickyFaults() {
		return getPeriodicStatus0().stickyFaults;
	}

	/**
	 * Get the value of a specific fault
	 *
	 * @param faultID The ID of the fault to retrive
	 *
	 * @return True if the fault with the given ID occurred.
	 */
	public boolean getFault(FaultID faultID) {
		short val = (short) (getFaults() & (1 << faultID.value));
		return val != 0;
	}

	/**
	 * Get the value of a specific sticky fault
	 *
	 * @param faultID The ID of the sticky fault to retrive
	 *
	 * @return True if the sticky fault with the given ID occurred.
	 */
	public boolean getStickyFault(FaultID faultID) {
		short val = (short) (getStickyFaults() & (1 << faultID.value));
		return val != 0;
	}

	/**
	 * @return The voltage fed into the motor controller.
	 */
	public double getBusVoltage() {
		return getPeriodicStatus1().busVoltage;
	}

	/**
	 * @return The motor controller's applied output duty cycle.
	 */
	public double getAppliedOutput() {
		return getPeriodicStatus0().appliedOutput;
	}

	/**
	 * @return The motor controller's output current in Amps.
	 */
	public double getOutputCurrent() {
		return getPeriodicStatus1().outputCurrent;
	}

	/**
	 * @return The motor temperature in Celsius.
	 */
	public double getMotorTemperature() {
		double val = (double) getPeriodicStatus1().motorTemperature;
		if (val < 0) {
			val = val + 256;
		}
		return val;
	}

	/**
	 * Clears all sticky faults.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError clearFaults() {
		CANData frame = new CANData();
		try {
			m_can.writePacket(frame.data, CANSparkMaxFrames.CMD_API_CLEAR_FAULTS);
			return CANError.kOK;
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}
	}

	/**
	 * Writes all settings to flash.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError burnFlash() {
		CANData frame = new CANData();
		frame.data[0] = (byte) 0xA3;
		frame.data[1] = (byte) 0x3A;
		try {
			m_can.writePacket(frame.data, CANSparkMaxFrames.CMD_API_BURN_FLASH);
			return CANError.kOK;
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}
	}

	/**
	 * Sets timeout for sending CAN messages.
	 *
	 * @param milliseconds The timeout in milliseconds.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 */
	public CANError setCANTimeout(int milliseconds) {
		m_canTimeoutMs = milliseconds;

		if (milliseconds < 0) {
			return CANError.kError;
		} else {
			return CANError.kOK;
		}
	}
}
