// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max steer motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSparkMAX implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase steerSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder steerEncoder;
  private final CANcoder absoluteEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController steerController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> steerPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer steerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOSparkMAX(int module) {
    zeroRotation = Rotation2d.kZero;
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Ports.FRONT_LEFT_DRIVE_MOTOR_PORT;
              case 1 -> Ports.FRONT_RIGHT_DRIVE_MOTOR_PORT;
              case 2 -> Ports.BACK_LEFT_DRIVE_MOTOR_PORT;
              case 3 -> Ports.BACK_RIGHT_DRIVE_MOTOR_PORT;
              default -> 0;
            },
            MotorType.kBrushless);
    steerSpark =
        new SparkMax(
            switch (module) {
              case 0 -> Ports.FRONT_LEFT_STEER_MOTOR_PORT;
              case 1 -> Ports.FRONT_RIGHT_STEER_MOTOR_PORT;
              case 2 -> Ports.BACK_LEFT_STEER_MOTOR_PORT;
              case 3 -> Ports.BACK_RIGHT_STEER_MOTOR_PORT;
              default -> 0;
            },
            MotorType.kBrushless);
    absoluteEncoder = new CANcoder(
        switch (module) {
            case 0 -> Ports.FRONT_LEFT_ABSOLUTE_ENCODER_PORT;
            case 1 -> Ports.FRONT_RIGHT_ABSOLUTE_ENCODER_PORT;
            case 2 -> Ports.BACK_LEFT_ABSOLUTE_ENCODER_PORT;
            case 3 -> Ports.BACK_RIGHT_ABSOLUTE_ENCODER_PORT;
            default -> 0;
        }
    );
    driveEncoder = driveSpark.getEncoder();
    steerEncoder = steerSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    steerController = steerSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Config.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(Physical.driveEncoderPositionFactor)
        .velocityConversionFactor(Physical.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Tunings.driveKp, 0.0, Tunings.driveKd);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure steer motor
    var steerConfig = new SparkMaxConfig();
    steerConfig
        .inverted(Config.steerInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Config.steerMotorCurrentLimit)
        .voltageCompensation(12.0);
    steerConfig
        .encoder
        .positionConversionFactor(Physical.steerEncoderPositionFactor)
        .velocityConversionFactor(Physical.steerEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(Tunings.steerKp, 0.0, Tunings.steerKd);
    steerConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / Config.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        steerSpark,
        5,
        () ->
            steerSpark.configure(
                steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    steerPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(steerSpark, steerEncoder::getPosition);

    // Reset Encoders
    driveEncoder.setPosition(0.0);
    steerEncoder.setPosition(
        absoluteEncoder.getPosition().getValueAsDouble()
    );
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update steer inputs
    sparkStickyFault = false;
    ifOk(
        steerSpark,
        steerEncoder::getPosition,
        (value) -> inputs.steerPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(steerSpark, steerEncoder::getVelocity, (value) -> inputs.steerVelocityRadPerSec = value);
    ifOk(
        steerSpark,
        new DoubleSupplier[] {steerSpark::getAppliedOutput, steerSpark::getBusVoltage},
        (values) -> inputs.steerAppliedVolts = values[0] * values[1]);
    ifOk(steerSpark, steerSpark::getOutputCurrent, (value) -> inputs.steerCurrentAmps = value);
    inputs.steerConnected = steerConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        steerPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    steerSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = Tunings.driveKs * Math.signum(velocityRadPerSec) + Tunings.driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), -Math.PI, Math.PI);
    steerController.setSetpoint(setpoint, ControlType.kPosition);
  }
}