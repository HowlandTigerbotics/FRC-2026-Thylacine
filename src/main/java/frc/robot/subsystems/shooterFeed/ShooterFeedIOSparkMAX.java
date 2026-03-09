// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import static frc.robot.subsystems.shooterFeed.ShooterFeedConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class ShooterFeedIOSparkMAX implements ShooterFeedIO {
    // Hardware objects
    private final SparkBase feedSpark;
    private final RelativeEncoder feedEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController feedController;

    // Connection debouncers
    private final Debouncer feedConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public ShooterFeedIOSparkMAX() {
        feedSpark = new SparkMax(Ports.FEED_PORT_ID, MotorType.kBrushless);
        feedEncoder = feedSpark.getEncoder();
        feedController = feedSpark.getClosedLoopController();
        configureMotors();
    }

    private void configureMotors() {
        // Configure feed motor
        var feedConfig = new SparkMaxConfig();
        feedConfig
                .idleMode(IdleMode.kCoast) // IMPORTANT: Coast because bangbang controller
                .inverted(Config.kFeedInverted)
                .smartCurrentLimit(Config.kSmartCurrentLimit)
                .voltageCompensation(Config.kNominalVoltage);
        feedConfig.encoder
                .positionConversionFactor(Physical.feedEncoderPositionFactor)
                .velocityConversionFactor(Physical.feedEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        feedConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                ;//.pid(Tunings.feedKp, 0, Tunings.feedKd);
        feedConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.kUpdateFrequency))
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
    public void updateInputs(ShooterFeedIOInputs inputs) {
        // Update feed inputs
        sparkStickyFault = false;
        ifOk(feedSpark, feedEncoder::getPosition, (value) -> inputs.feedPositionRad = value);
    ifOk(feedSpark, feedEncoder::getVelocity, (value) -> inputs.feedVelocityRadPerSec = value);
    ifOk(
        feedSpark,
        new DoubleSupplier[] {feedSpark::getAppliedOutput, feedSpark::getBusVoltage},
        (values) -> inputs.feedAppliedVolts = values[0] * values[1]);
    ifOk(feedSpark, feedSpark::getOutputCurrent, (value) -> inputs.feedCurrentAmps = value);
    inputs.feedConnected = feedConnectedDebouncer.calculate(!sparkStickyFault);
    }

  @Override
  public void setFeedSpeed(double speed) {
    feedSpark.set(speed);
  }
}
