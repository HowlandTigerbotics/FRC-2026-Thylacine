// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeFeed;

import static frc.robot.subsystems.intakeFeed.IntakeFeedConstants.*;
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
public class IntakeFeedIOSparkMAX implements IntakeFeedIO{
  // Hardware objects
  private final SparkBase feedSpark;
  private final RelativeEncoder feedEncoder;

  // Connection debouncers
  private final Debouncer feedConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  
  public IntakeFeedIOSparkMAX() {
    feedSpark = new SparkMax(Ports.INTAKE_FEED_MOTOR_PORT, MotorType.kBrushless);

    feedEncoder = feedSpark.getEncoder();

    configureMotors();
  }

  private void configureMotors() {
    // TODO: Update the constants later
    var feedConfig = new SparkMaxConfig();
    feedConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    feedConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    feedConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0.0, 0);
    feedConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        feedSpark,
        5,
        () -> feedSpark.configure(
            feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(feedSpark, 5, () -> feedEncoder.setPosition(0.0));

  }

  @Override
  public void updateInputs(IntakeFeedIOInputs inputs) {
    // Update feed inputs
    sparkStickyFault = false;
    ifOk(feedSpark, feedEncoder::getPosition, (value) -> inputs.feedPositionRad = value);
    ifOk(feedSpark, feedEncoder::getVelocity, (value) -> inputs.feedVelocityRadPerSec = value);
    ifOk(
        feedSpark,
        new DoubleSupplier[] { feedSpark::getAppliedOutput, feedSpark::getBusVoltage },
        (values) -> inputs.feedAppliedVolts = values[0] * values[1]);
    ifOk(feedSpark, feedSpark::getOutputCurrent, (value) -> inputs.feedCurrentAmps = value);
    inputs.feedConnected = feedConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setFeedSpeed(double speed) {
    feedSpark.set(speed);
  }
}
