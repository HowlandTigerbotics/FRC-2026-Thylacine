// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeRoller;

import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.*;
import static frc.robot.util.SparkUtil.*;

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

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class IntakeRollerIOSparkMAX implements IntakeRollerIO{
  // Hardware objects
  private final SparkBase intakeSpark;
  private final RelativeEncoder intakeEncoder;

  private final SparkClosedLoopController intakeController;

  // Connection debouncers
  private final Debouncer intakeConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  
  public IntakeRollerIOSparkMAX() {
    intakeSpark = new SparkMax(Ports.INTAKE_MOTOR_PORT, MotorType.kBrushless);

    intakeEncoder = intakeSpark.getEncoder();

    intakeController = intakeSpark.getClosedLoopController();

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
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
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

   
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    intakeSpark.setVoltage(output);
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
  public void setIntakePercent(double percent) {
    intakeSpark.set(percent);
  }
}
