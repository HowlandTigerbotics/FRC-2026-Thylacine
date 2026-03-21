// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakePosition;

import static frc.robot.subsystems.intakePosition.IntakeRollerPosition.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class IntakePositionIOSparkMAX implements IntakePositionIO{
  // Hardware objects
  private final SparkBase positionSpark;
  private final RelativeEncoder positionEncoder;

  private final DigitalInput limitSwitch;

  // Closed loop controllers
  private final SparkClosedLoopController positionController;

  // Connection debouncers
  private final Debouncer positionConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakePositionIOSparkMAX() {
    positionSpark = new SparkMax(Ports.POSITION_MOTOR_PORT, MotorType.kBrushless);

    positionEncoder = positionSpark.getEncoder();
    
    positionController = positionSpark.getClosedLoopController();

    limitSwitch = new DigitalInput(2);

    configureMotors();
  }

  private void configureMotors() {
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
  public void updateInputs(IntakePositionIOInputs inputs) {
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

    inputs.limitSwitch = !limitSwitch.get();
  }

  @Override
  public void setPositionOpenLoop(double output) {
    positionSpark.setVoltage(output);
  }


  @Override
  public void setPositionPosition(Rotation2d rotation) {
    double setpoint = MathUtil.inputModulus(
        rotation.getRadians(), -Math.PI, Math.PI);
    positionController.setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void setPositionPercent(double percent) {
    positionSpark.set(percent);
  }
}
