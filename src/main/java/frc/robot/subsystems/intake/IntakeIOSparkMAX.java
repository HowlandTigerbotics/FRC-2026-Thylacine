// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.Queue;
import java.util.function.DoubleSupplier;

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

/** Add your docs here. */
public class IntakeIOSparkMAX implements IntakeIO{
  // Hardware objects
  private final SparkBase intakeSpark;
  private final SparkBase positionSpark;
  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder positionEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController intakeController;
  private final SparkClosedLoopController positionController;

  // Connection debouncers
  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer positionConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOSparkMAX() {
    intakeSpark = new SparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    positionSpark = new SparkMax(Ports.POSITION_MOTOR_PORT, MotorType.kBrushless);

    intakeEncoder = intakeSpark.getEncoder();
    positionEncoder = positionSpark.getEncoder();
    intakeController = intakeSpark.getClosedLoopController();
    positionController = positionSpark.getClosedLoopController();

    configureMotors();
  }

  private void configureMotors() {
    // Configure intake motor
    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(Config.intakeMotorCurrentLimit)
        .voltageCompensation(12.0);
    intakeConfig.encoder
        .positionConversionFactor(Physical.intakeEncoderPositionFactor)
        .velocityConversionFactor(Physical.intakeEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Tunings.intakeKp, 0.0, Tunings.intakeKd);
    intakeConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeSpark,
        5,
        () -> intakeSpark.configure(
            intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(intakeSpark, 5, () -> intakeEncoder.setPosition(0.0));

    // Configure position motor
    var positionConfig = new SparkMaxConfig();
    positionConfig
        .inverted(Config.positionInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Config.positionMotorCurrentLimit)
        .voltageCompensation(12.0);
    positionConfig.encoder
        .positionConversionFactor(Physical.positionEncoderPositionFactor)
        .velocityConversionFactor(Physical.positionEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    positionConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(Tunings.positionKp, 0.0, Tunings.positionKd);
    positionConfig.signals // TODO: Changed this so no longer uses abs.
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        positionSpark,
        5,
        () -> positionSpark.configure(
            positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    // Reset Encoders
    tryUntilOk(positionSpark, 5, () -> positionEncoder.setPosition(0.0));

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update intake inputs
    sparkStickyFault = false;
    ifOk(intakeSpark, intakeEncoder::getPosition, (value) -> inputs.intakePositionRad = value);
    ifOk(intakeSpark, intakeEncoder::getVelocity, (value) -> inputs.intakeVelocityRadPerSec = value);
    ifOk(
        intakeSpark,
        new DoubleSupplier[] { intakeSpark::getAppliedOutput, intakeSpark::getBusVoltage },
        (values) -> inputs.intakeAppliedVolts = values[0] * values[1]);
    ifOk(intakeSpark, intakeSpark::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
    inputs.intakeConnected = intakeConnectedDebounce.calculate(!sparkStickyFault);

    // Update position inputs
    sparkStickyFault = false;
    ifOk(
        positionSpark,
        positionEncoder::getPosition,
        (value) -> inputs.positionPosition = new Rotation2d(value).minus(Rotation2d.kZero));
    ifOk(positionSpark, positionEncoder::getVelocity, (value) -> inputs.positionVelocityRadPerSec = value);
    ifOk(
        positionSpark,
        new DoubleSupplier[] { positionSpark::getAppliedOutput, positionSpark::getBusVoltage },
        (values) -> inputs.positionAppliedVolts = values[0] * values[1]);
    ifOk(positionSpark, positionSpark::getOutputCurrent, (value) -> inputs.positionCurrentAmps = value);
    inputs.positionConnected = positionConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    intakeSpark.setVoltage(output);
  }

  @Override
  public void setPositionOpenLoop(double output) {
    positionSpark.setVoltage(output);
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    double ffVolts = Tunings.intakeKs * Math.signum(velocityRadPerSec) + Tunings.intakeKv * velocityRadPerSec;
    intakeController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPositionPosition(Rotation2d rotation) {
    double setpoint = MathUtil.inputModulus(
        rotation.getRadians(), -Math.PI, Math.PI);
    positionController.setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSpark.set(speed);
  }

  @Override
  public void setPositionSpeed(double speed) {
    positionSpark.set(speed);
  }
}
