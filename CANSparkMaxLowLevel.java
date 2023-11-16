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

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;

import java.util.Optional;
import java.util.Random;

import com.revrobotics.CANSparkMaxFrames.FirmwareIn;
import com.revrobotics.CANSparkMaxFrames.FollowerOut;
import com.revrobotics.CANSparkMaxFrames.GetParamIn;
import com.revrobotics.CANSparkMaxFrames.SetParamOut;
import com.revrobotics.CANSparkMaxFrames.SetpointOut;
import com.revrobotics.CANSparkMaxFrames.StatusConfigOut;
import com.revrobotics.jni.CANHeartbeatJNI;
import com.revrobotics.CANSparkMaxFrames.Status0In;
import com.revrobotics.CANSparkMaxFrames.Status1In;
import com.revrobotics.CANSparkMaxFrames.Status2In;

public abstract class CANSparkMaxLowLevel implements SpeedController {
	public static final byte kNumFirmwareRetries = 10;
	public static final int kDefaultCANTimeoutMs = 20;
	public static final int kDefaultStatus0PeriodMs = 10;
	public static final int kDefaultStatus1PeriodMs = 20;
	public static final int kDefaultStatus2PeriodMs = 50;
	public static final int kMinFirmwareVersion = 0x1000179;

	public enum MotorType {
		kBrushed(0), kBrushless(1);

		@SuppressWarnings("MemberName")
		public final int value;

		MotorType(int value) {
			this.value = value;
		}
	}

	public enum ParameterStatus {
		kOK(0), kInvalidID(1), kMismatchType(2), kAccessMode(3), kInvalid(4), kNotImplementedDeprecated(5);

		@SuppressWarnings("MemberName")
		public final int value;

		ParameterStatus(int value) {
			this.value = value;
		}
	}

	public enum ConfigParameter {
		kCanID(0), kInputMode(1), kMotorType(2), kCommAdvance(3), kSensorType(4), kCtrlType(5), kIdleMode(6),
		kInputDeadband(7), kFirmwareVer(8), kHallOffset(9), kPolePairs(10), kCurrentChop(11), kCurrentChopCycles(12),
		kP_0(13), kI_0(14), kD_0(15), kF_0(16), kIZone_0(17), kDFilter_0(18), kOutputMin_0(19), kOutputMax_0(20),
		kP_1(21), kI_1(22), kD_1(23), kF_1(24), kIZone_1(25), kDFilter_1(26), kOutputMin_1(27), kOutputMax_1(28),
		kP_2(29), kI_2(30), kD_2(31), kF_2(32), kIZone_2(33), kDFilter_2(34), kOutputMin_2(35), kOutputMax_2(36),
		kP_3(37), kI_3(38), kD_3(39), kF_3(40), kIZone_3(41), kDFilter_3(42), kOutputMin_3(43), kOutputMax_3(44),
		kReserved(45), kOutputRatio(46), kSerialNumberLow(47), kSerialNumberMid(48), kSerialNumberHigh(49),
		kLimitSwitchFwdPolarity(50), kLimitSwitchRevPolarity(51), kHardLimitFwdEn(52), kHardLimitRevEn(53),
		kSoftLimitFwdEn(54), kSoftLimitRevEn(55), kOpenLoopRampRate(56), kFollowerID(57), kFollowerConfig(58),
		kSmartCurrentStallLimit(59), kSmartCurrentFreeLimit(60), kSmartCurrentConfig(61), kSmartCurrentReserved(62),
		kMotorKv(63), kMotorR(64), kMotorL(65), kMotorRsvd1(66), kMotorRsvd2(67), kMotorRsvd3(68),
		kEncoderCountsPerRev(69), kEncoderAverageDepth(70), kEncoderSampleDelta(71), kEncoderRsvd0(72),
		kEncoderRsvd1(73), kVoltageCompMode(74), kCompensatedNominalVoltage(75), kSmartMotionMaxVelocity_0(76),
		kSmartMotionMaxAccel_0(77), kSmartMotionMinVelOutput_0(78), kSmartMotionAllowedClosedLoopError_0(79),
		kSmartMotionAccelStrategy_0(80), kSmartMotionMaxVelocity_1(81), kSmartMotionMaxAccel_1(82),
		kSmartMotionMinVelOutput_1(83), kSmartMotionAllowedClosedLoopError_1(84), kSmartMotionAccelStrategy_1(85),
		kSmartMotionMaxVelocity_2(86), kSmartMotionMaxAccel_2(87), kSmartMotionMinVelOutput_2(88),
		kSmartMotionAllowedClosedLoopError_2(89), kSmartMotionAccelStrategy_2(90), kSmartMotionMaxVelocity_3(91),
		kSmartMotionMaxAccel_3(92), kSmartMotionMinVelOutput_3(93), kSmartMotionAllowedClosedLoopError_3(94),
		kSmartMotionAccelStrategy_3(95), kIMaxAccum_0(96), kSlot3Placeholder1_0(97), kSlot3Placeholder2_0(98),
		kSlot3Placeholder3_0(99), kIMaxAccum_1(100), kSlot3Placeholder1_1(101), kSlot3Placeholder2_1(102),
		kSlot3Placeholder3_1(103), kIMaxAccum_2(104), kSlot3Placeholder1_2(105), kSlot3Placeholder2_2(106),
		kSlot3Placeholder3_2(107), kIMaxAccum_3(108), kSlot3Placeholder1_3(109), kSlot3Placeholder2_3(110),
		kSlot3Placeholder3_3(111), kPositionConversionFactor(112), kVelocityConversionFactor(113),
		kClosedLoopRampRate(114),;

		@SuppressWarnings("MemberName")
		public final int value;

		ConfigParameter(int value) {
			this.value = value;
		}

		public static ConfigParameter fromId(int id) {
			for (ConfigParameter type : values()) {
				if (type.value == id) {
					return type;
				}
			}
			return null;
		}

	}

	public enum ParameterType {
		kInt32(0), kUint32(1), kFloat32(2), kBool(3);

		@SuppressWarnings("MemberName")
		public final int value;

		ParameterType(int value) {
			this.value = value;
		}
	}

	public enum PeriodicFrame {
		kStatus0(0), kStatus1(1), kStatus2(2);

		@SuppressWarnings("MemberName")
		public final int value;

		PeriodicFrame(int value) {
			this.value = value;
		}
	}

	public class PeriodicStatus0 {
		public double appliedOutput;
		public short faults;
		public short stickyFaults;
		public byte idleMode;
		public MotorType motorType;
		public boolean isFollower;
	}

	public class PeriodicStatus1 {
		public double sensorVelocity;
		public byte motorTemperature;
		public double busVoltage;
		public double outputCurrent;
	}

	public class PeriodicStatus2 {
		public double sensorPosition;
		public double iAccum;
	}

	/**
	 * Create a new SPARK MAX Controller
	 *
	 * @param deviceID The device ID.
	 * @param type     The motor type connected to the controller. Brushless motors
	 *                 must be connected to their matching color and the hall sensor
	 *                 plugged in. Brushed motors must be connected to the Red and
	 *                 Black terminals only.
	 */
	public CANSparkMaxLowLevel(int deviceID, MotorType type) {
		m_deviceID = deviceID;
		try {
			m_can = new CAN(deviceID, HAL_CAN_Man_kREV, HAL_CAN_Dev_kMotorController);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
		}

		m_firmwareVersion = getFirmwareVersion();

		m_canTimeoutMs = kDefaultCANTimeoutMs;

		m_status0PeriodMs = kDefaultStatus0PeriodMs;
		m_status1PeriodMs = kDefaultStatus1PeriodMs;
		m_status2PeriodMs = kDefaultStatus2PeriodMs;

		m_activeSetpointApi = -1;

		m_inverted = false;

		if (m_firmwareVersion == 0) {
			DriverStation.reportError(String.join("", "Unable to retrieve SPARK MAX firmware version ",
					"please verify the deviceID field matches the ", "configured CAN ID of the controller, and that ",
					"the controller is connected to the CAN Bus."), false);
		} else if (m_firmwareVersion < kMinFirmwareVersion) {
			DriverStation.reportError(String.join("", "The firmware on SPARK MAX with CAN ID: ",
					Integer.toString(m_deviceID), " is too old and needs to be updated. Refer ",
					"to www.RevRobotics.com/sparkmax for details."), false);
		}

		setMotorType(type);

		CANHeartbeatJNI.RegisterDevice(deviceID);

		HAL.report(tResourceType.kResourceType_RevSparkMaxCAN, deviceID);
	}

	/**
	 * Get the firmware version of the SPARK MAX.
	 *
	 * @return uint32_t Firmware version integer. Value is represented as 4 bytes,
	 *         Major.Minor.Build H.Build L
	 *
	 */
	public int getFirmwareVersion() {
		FirmwareIn frame = new FirmwareIn();
		int firmwareVersionFlattened;
		CANData data = new CANData();

		try {
			m_can.writePacket(new byte[] { (byte) 0 }, CANSparkMaxFrames.CMD_API_FIRMWARE);

			int retries = 0;
			while (retries < kNumFirmwareRetries) {
				if (m_can.readPacketTimeout(CANSparkMaxFrames.CMD_API_FIRMWARE, 30, data)) {
					// Firmware read successful
					break;
				}
				retries++;
			}

			if (retries == kNumFirmwareRetries) {
				// Firmware version not recieved
				return 0;
			}
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return 0;
		}

		frame.Deserialize(data.data);

		short firmwareBuild = frame.firmwareBuild;

		firmwareVersionFlattened = (int) (frame.firmwareMajor) << 24 | (int) (frame.firmwareMinor) << 16
				| firmwareBuild;

		StringBuilder firmwareString = new StringBuilder();
		firmwareString.append("v").append((int) frame.firmwareMajor).append(".").append((int) frame.firmwareMinor)
				.append(".").append(firmwareBuild);

		m_firmwareString = firmwareString.toString();

		return firmwareVersionFlattened;
	}

	/**
	 * Get the firmware version of the SPARK MAX as a string.
	 *
	 * @return std::string Human readable firmware version string
	 *
	 */
	public String getFirmwareString() {
		return m_firmwareString;
	}

	/**
	 * Get the unique serial number of the SPARK MAX. Not currently available.
	 *
	 * @return byte[] Vector of bytes representig the unique serial number
	 *
	 */
	public byte[] getSerialNumber() {
		return new byte[0];
	}

	/**
	 * Get the configured Device ID of the SPARK MAX.
	 *
	 * @return int device ID
	 *
	 */
	public int getDeviceId() {
		return m_deviceID;
	}

	/**
	 * Set the motor type connected to the SPARK MAX.
	 *
	 * This uses the Set Parameter API and should be used infrequently. The
	 * parameter does not presist unless burnFlash() is called. The recommended
	 * method to configure this parameter is to use the SPARK MAX GUI to tune and
	 * save parameters.
	 *
	 * @param type The type of motor connected to the controller. Brushless motors
	 *             must be connected to their matching color and the hall sensor
	 *             plugged in. Brushed motors must be connected to the Red and Black
	 *             terminals only.
	 *
	 * @return CANError Set to CANError::kOk if successful
	 *
	 */
	public CANError setMotorType(MotorType type) {
		ParameterStatus status = setParameter(ConfigParameter.kMotorType, type.value);
		if (status == ParameterStatus.kOK) {
			return CANError.kOK;
		} else {
			return CANError.kError;
		}
	}

	/**
	 * Get the motor type setting for the SPARK MAX.
	 *
	 * This uses the Get Parameter API and should be used infrequently. This
	 * function uses a non-blocking call and will return a cached value if the
	 * parameter is not returned by the timeout. The timeout can be changed by
	 * calling SetCANTimeout(int milliseconds)
	 *
	 * @return MotorType Motor type setting
	 *
	 */
	public MotorType getMotorType() {
		Optional<Integer> value;
		value = getParameterInt(ConfigParameter.kMotorType);
		if (value.isPresent()) {
			return MotorType.values()[value.get()];
		} else {
			return MotorType.kBrushless;
		}
	}

	/**
	 * Set the rate of transmission for periodic frames from the SPARK MAX
	 *
	 * Each motor controller sends back three status frames with different data at
	 * set rates. Use this function to change the default rates.
	 *
	 * Defaults: Status0 - 10ms Status1 - 20ms Status2 - 50ms
	 *
	 * This value is not stored in the FLASH after calling burnFlash() and is reset
	 * on powerup.
	 *
	 * Refer to the SPARK MAX reference manual on details for how and when to
	 * configure this parameter.
	 *
	 * @param frameID  The frame ID can be one of PeriodicFrame type
	 * @param periodMs The rate the controller sends the frame to the controller.
	 *
	 * @return CANError Set to CANError.kOk if successful
	 *
	 */
	public CANError setPeriodicFramePeriod(PeriodicFrame frameID, int periodMs) {
		int apiID;

		if (frameID == PeriodicFrame.kStatus0) {
			apiID = CANSparkMaxFrames.CMD_API_STAT0;
			m_status0PeriodMs = periodMs;
		} else if (frameID == PeriodicFrame.kStatus1) {
			apiID = CANSparkMaxFrames.CMD_API_STAT1;
			m_status1PeriodMs = periodMs;
		} else if (frameID == PeriodicFrame.kStatus2) {
			apiID = CANSparkMaxFrames.CMD_API_STAT2;
			m_status2PeriodMs = periodMs;
		} else {
			return CANError.kError;
		}

		StatusConfigOut frame = new StatusConfigOut();

		frame.updateRate = (short) periodMs;
		try {
			m_can.writePacket(frame.Serialize(), apiID);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		return CANError.kOK;
	}

	public ParameterStatus setParameter(ConfigParameter parameterID, double value) {
		return setParameterCore(parameterID, ParameterType.kFloat32, CANSparkMaxFrames.packFloat32(value));
	}

	public ParameterStatus setParameter(ConfigParameter parameterID, int value) {
		return setParameterCore(parameterID, ParameterType.kUint32, value);
	}

	public ParameterStatus setParameter(ConfigParameter parameterID, boolean value) {
		return setParameterCore(parameterID, ParameterType.kBool, value ? 1 : 0);
	}

	public Optional<Double> getParameterDouble(ConfigParameter parameterID) {
		Optional<Integer> tmp = getParameterCore(parameterID, ParameterType.kFloat32);

		if (tmp.isPresent()) {
			return Optional.of(CANSparkMaxFrames.unpackFloat32(tmp.get()));
		} else {
			return Optional.empty();
		}
	}

	public Optional<Integer> getParameterInt(ConfigParameter parameterID) {
		return getParameterCore(parameterID, ParameterType.kUint32);
	}

	public Optional<Boolean> getParameterBoolean(ConfigParameter parameterID) {
		Optional<Integer> value = getParameterCore(parameterID, ParameterType.kBool);
		if (value.isPresent()) {
			return Optional.of((value.get() == 0) ? false : true);
		} else {
			return Optional.empty();
		}
	}

	CANError setFollow(FollowConfig follower) {
		final int kFollowFrameLength = 8;
		CANData canReturnFrame = new CANData();
		FollowerOut frame = new FollowerOut();
		frame.followerCfg = follower.config.getRaw();
		frame.followerID = follower.leaderArbId;
		try {
			m_can.writePacket(frame.Serialize(), CANSparkMaxFrames.CMD_API_SET_FOLLOWER);

			// Confim good setup checking response
			if (m_can.readPacketTimeout(CANSparkMaxFrames.CMD_API_SET_FOLLOWER, m_canTimeoutMs,
					canReturnFrame) == false) {
				return CANError.kTimeout;
			}
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		if (canReturnFrame.length < kFollowFrameLength) {
			return CANError.kError;
		}

		for (int i = 0; i < kFollowFrameLength; i++) {
			if (frame.Serialize()[i] != canReturnFrame.data[i]) {
				return CANError.kError;
			}
		}

		return CANError.kOK;
	}

	CANError setpointCommand(double value) {
		return setpointCommand(value, ControlType.kDutyCycle);
	}

	CANError setpointCommand(double value, ControlType ctrl) {
		return setpointCommand(value, ctrl, 0);
	}

	CANError setpointCommand(double value, ControlType ctrl, int pidSlot) {
		return setpointCommand(value, ctrl, pidSlot, 0);
	}

	CANError setpointCommand(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
		SetpointOut frame = new SetpointOut();
		int apiId = 0;

		if (ctrl == ControlType.kDutyCycle) {
			apiId = CANSparkMaxFrames.CMD_API_DC_SET;
		} else if (ctrl == ControlType.kPosition) {
			apiId = CANSparkMaxFrames.CMD_API_POS_SET;
		} else if (ctrl == ControlType.kVelocity) {
			apiId = CANSparkMaxFrames.CMD_API_SPD_SET;
		} else if (ctrl == ControlType.kVoltage) {
			apiId = CANSparkMaxFrames.CMD_API_VOLT_SET;
		} else if (ctrl == ControlType.kSmartMotion) {
			apiId = CANSparkMaxFrames.CMD_API_SMARTMOTION_SET;
		} else if (ctrl == ControlType.kCurrent) {
			apiId = CANSparkMaxFrames.CMD_API_CURRENT_SET;
		} else {
			return CANError.kError;
		}

		double shiftedArbFF = (arbFeedforward * 1024);
		short packedFF;

		if (shiftedArbFF > 32767) {
			packedFF = 32767;
		} else if (shiftedArbFF < -32767) {
			packedFF = -32767;
		} else {
			packedFF = (short) shiftedArbFF;
		}

		frame.auxSetpoint = packedFF;
		frame.setpoint = (float) value;
		frame.pidSlot = (byte) pidSlot;

		if (m_inverted) {
			frame.auxSetpoint *= -1;
			frame.setpoint *= -1;
		}

		try {
			m_activeSetpointApi = apiId;
			m_can.writePacket(frame.Serialize(), apiId);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		return CANError.kOK;
	}

	public CANError setEncPosition(double value) {
		SetParamOut frame = new SetParamOut();

		if(m_inverted) {
			value = value * -1;
		}

		frame.parameter = CANSparkMaxFrames.packFloat32(value);
		frame.parameterType = (byte) ParameterType.kFloat32.value;

		try {
			m_can.writePacket(frame.Serialize(), CANSparkMaxFrames.CMD_API_MECH_POS);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		return CANError.kOK;
	}

	public CANError setIAccum(double value) {
		SetParamOut frame = new SetParamOut();
		frame.parameter = CANSparkMaxFrames.packFloat32(value);
		frame.parameterType = (byte) ParameterType.kFloat32.value;

		try {
			m_can.writePacket(frame.Serialize(), CANSparkMaxFrames.CMD_API_I_ACCUM);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		return CANError.kOK;
	}

	/**
	 * Restore motor controller parameters to factory default until the next
	 * controller reboot
	 *
	 * @return CANError Set to CANError::kOk if successful
	 */
	public CANError restoreFactoryDefaults() {
		return restoreFactoryDefaults(false);
	}

	/**
	 * Restore motor controller parameters to factory default
	 *
	 * @param persist If true, burn the flash with the factory default parameters
	 *
	 * @return CANError Set to CANError::kOk if successful
	 */
	public CANError restoreFactoryDefaults(boolean persist) {
		SetParamOut frame = new SetParamOut();
		frame.parameter = persist ? 1 : 0;
		frame.parameterType = (byte) ParameterType.kBool.value;

		try {
			m_can.writePacket(frame.Serialize(), CANSparkMaxFrames.CMD_API_FACTORY_DEFAULT);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return CANError.kError;
		}

		return CANError.kOK;
	}

	public ParameterStatus setParameterCore(ConfigParameter parameterID, ParameterType type, int value) {
		SetParamOut frame = new SetParamOut();
		frame.parameter = value;
		frame.parameterType = (byte) type.value;

		try {
			m_can.writePacket(frame.Serialize(), CANSparkMaxFrames.CMD_API_PARAM_ACCESS | parameterID.value);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return ParameterStatus.kInvalid;
		}
		return ParameterStatus.kOK;
	}

	public Optional<Integer> getParameterCore(ConfigParameter parameterID, ParameterType expectedType) {
		// if (getParameterType(parameterID) != expectedType) {
		// return ParameterStatus.kMismatchType;
		// }

		try {
			// Request parameter
			int apiID = CANSparkMaxFrames.CMD_API_PARAM_ACCESS | parameterID.value;
			m_can.writePacket(new byte[0], apiID);

			// Wait for reply containing parameter
			CANData data = new CANData();
			if (!m_can.readPacketTimeout(apiID, m_canTimeoutMs, data)) {
				return Optional.empty();
			}

			// Extract parameter from payload
			GetParamIn payload = new GetParamIn();
			payload.Deserialize(data.data);

			return Optional.of(payload.parameter0);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return Optional.empty();
		}
	}

	public ParameterType getParameterType(ConfigParameter parameterID) {
		// if (getParameterType(parameterID) != expectedType) {
		// return ParameterStatus.kMismatchType;
		// }
		try {
			// Request parameter
			int apiID = CANSparkMaxFrames.CMD_API_PARAM_ACCESS | parameterID.value;
			m_can.writePacket(new byte[0], apiID);

			// Wait for reply containing parameter
			CANData data = new CANData();
			if (!m_can.readPacketTimeout(apiID, m_canTimeoutMs, data)) {
				return ParameterType.values()[ParameterStatus.kInvalid.value];
			}

			// Extract parameter from payload
			GetParamIn payload = new GetParamIn();
			payload.Deserialize(data.data);

			return ParameterType.values()[payload.parameter0];
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return ParameterType.values()[ParameterStatus.kInvalid.value];
		}
	}

	protected PeriodicStatus0 getPeriodicStatus0() {
		Status0In statusFrame0 = new Status0In();
		PeriodicStatus0 status0 = new PeriodicStatus0();
		CANData frame = new CANData();
		try {
			m_can.readPeriodicPacket(CANSparkMaxFrames.CMD_API_STAT0, m_status0PeriodMs * 2 + m_canTimeoutMs,
					m_status0PeriodMs, frame);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return status0;
		}
		statusFrame0.Deserialize(frame.data);

		status0.appliedOutput = ((double) statusFrame0.appliedOutput) / 32767;
		if(m_inverted == true) {
			status0.appliedOutput *= -1;
		}
		status0.faults = statusFrame0.faults;
		status0.stickyFaults = statusFrame0.stickyFaults;
		status0.idleMode = statusFrame0.idleMode;
		// status0.motorType = statusFrame0.mtrType;
		status0.isFollower = statusFrame0.isFollower != 0;

		return status0;
	}

	protected PeriodicStatus1 getPeriodicStatus1() {
		Status1In statusFrame1 = new Status1In();
		PeriodicStatus1 status1 = new PeriodicStatus1();
		CANData frame = new CANData();
		try {
			m_can.readPeriodicPacket(CANSparkMaxFrames.CMD_API_STAT1, m_status1PeriodMs * 2 + m_canTimeoutMs,
					m_status1PeriodMs, frame);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return status1;
		}
		statusFrame1.Deserialize(frame.data);

		status1.busVoltage = (double) statusFrame1.mtrVoltage / 128;
		status1.outputCurrent = (double) statusFrame1.mtrCurrent / 32;
		status1.motorTemperature = statusFrame1.mtrTemp;
		status1.sensorVelocity = (double) statusFrame1.sensorVel;
		if(m_inverted == true) {
			status1.sensorVelocity *= -1;
		}
		return status1;
	}

	protected PeriodicStatus2 getPeriodicStatus2() {
		Status2In statusFrame2 = new Status2In();
		PeriodicStatus2 status2 = new PeriodicStatus2();
		CANData frame = new CANData();
		try {
			m_can.readPeriodicPacket(CANSparkMaxFrames.CMD_API_STAT2, m_status2PeriodMs * 2 + m_canTimeoutMs,
					m_status2PeriodMs, frame);
		} catch (Exception ignored) {
			notifyOnCANDisconnect();
			return status2;
		}
		statusFrame2.Deserialize(frame.data);

		status2.sensorPosition = statusFrame2.sensorPos;
		status2.iAccum = statusFrame2.iAccum;
		if(m_inverted == true) {
			status2.sensorPosition *= -1;
			status2.iAccum *= -1;
		}
		return status2;
	}

	protected static class FollowConfig {
		int leaderArbId;
		Config config;

		public static class Config {
			public int rsvd1;
			public int invert;
			public int rsvd2;
			public int predefined;

			public int getRaw() {
				return (rsvd1 & 0x3FFFF) | (invert & 0x1) << 18 | (rsvd2 & 0x1F) << 19 | (predefined & 0xFF) << 24;
			}
		}

		FollowConfig() {
			config = new Config();
		}
	}

	protected static void notifyOnCANDisconnect() {
		String errMsg = "Error! CAN BUS suffered a fatal unrecoverable error. Most likely a physical disconnect. Check your wires!";
		System.out.println(errMsg);
		DriverStation.reportError(errMsg, false);
	}

	protected CAN m_can;
	protected int m_controlPeriodMs;
	protected int m_canTimeoutMs;
	protected boolean m_inverted;

	private final int m_deviceID;

	private int m_status0PeriodMs;
	private int m_status1PeriodMs;
	private int m_status2PeriodMs;

	private int m_activeSetpointApi;

	private int m_firmwareVersion;
	private String m_firmwareString;

	// FIXME: remove when vendor and product ID are added to image
	static final int HAL_CAN_Man_kREV = 5;
	static final int HAL_CAN_Dev_kMotorController = 2;

	static {
		startDaemonThread(() -> heartbeatFunc());
	}

	private static void heartbeatFunc() {
		final byte kHeartbeatFramePeriodMs = 50;

		CANHeartbeatJNI.HeartbeatInit();

		while (true) {
			try {
				CANHeartbeatJNI.RunHeartbeat();
			} catch (Exception e) {
			}
			try {
				Thread.sleep(kHeartbeatFramePeriodMs);
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
	}

	private static Thread startDaemonThread(Runnable target) {
		Thread inst = new Thread(target);
		inst.setDaemon(true);
		inst.start();
		return inst;
	}
}
