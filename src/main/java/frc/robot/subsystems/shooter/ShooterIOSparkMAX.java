// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
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
public class ShooterIOSparkMAX implements ShooterIO{
    // Hardware objects
    private final SparkBase shooterSpark;
    private final SparkBase feedSpark;
    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder feedEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController shooterController;

    // Connection debouncers
    private final Debouncer shooterConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public ShooterIOSparkMAX() {
        shooterSpark = new SparkMax(Ports.SHOOTER_PORT_ID, MotorType.kBrushless);
        shooterEncoder = shooterSpark.getEncoder();
        shooterController = shooterSpark.getClosedLoopController();

        feedSpark = new SparkMax(Ports.FEED_PORT_ID, MotorType.kBrushless);
        feedEncoder = feedSpark.getEncoder();
        configureMotors();
    }

    private void configureMotors() {
        // Configure shooter motor
        var shooterConfig = new SparkMaxConfig();
        shooterConfig
                .idleMode(IdleMode.kCoast) // IMPORTANT: Coast because bangbang controller
                .inverted(Config.shooterInverted)
                .smartCurrentLimit(Config.kSmartCurrentLimit)
                .voltageCompensation(Config.kNominalVoltage);
        shooterConfig.encoder
                .positionConversionFactor(Physical.shooterEncoderPositionFactor)
                .velocityConversionFactor(Physical.shooterEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        shooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Tunings.shooterKp, 0, Tunings.shooterKd);
        shooterConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.kUpdateFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                shooterSpark,
                5,
                () -> shooterSpark.configure(
                        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(shooterSpark, 5, () -> shooterEncoder.setPosition(0.0));    
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update shooter inputs
        sparkStickyFault = false;
        ifOk(shooterSpark, shooterEncoder::getPosition, (value) -> inputs.shooterPositionRad = value);
    ifOk(shooterSpark, shooterEncoder::getVelocity, (value) -> inputs.shooterVelocityRadPerSec = value);
    ifOk(
        shooterSpark,
        new DoubleSupplier[] {shooterSpark::getAppliedOutput, shooterSpark::getBusVoltage},
        (values) -> inputs.shooterAppliedVolts = values[0] * values[1]);
    ifOk(shooterSpark, shooterSpark::getOutputCurrent, (value) -> inputs.shooterCurrentAmps = value);
    inputs.shooterConnected = shooterConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    double ffVolts = Tunings.shooterKs * Math.signum(velocityRadPerSec) + Tunings.shooterKv * velocityRadPerSec;
    shooterController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterSpark.set(speed);
  }
}
