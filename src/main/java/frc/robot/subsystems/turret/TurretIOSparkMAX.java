// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
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

/** Add your docs here. */
public class TurretIOSparkMAX implements TurretIO {
    // Hardware objects
    private final SparkBase turretSpark;
    private final RelativeEncoder turretEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController;

    // Connection debouncers
    private final Debouncer turretConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public TurretIOSparkMAX() {
        turretSpark = new SparkMax(Ports.TURRET_PORT_ID, MotorType.kBrushless);
        turretEncoder = turretSpark.getEncoder();
        turretController = turretSpark.getClosedLoopController();

        configureMotors();
    }

    private void configureMotors() {
        // Configure drive motor
        var turretConfig = new SparkMaxConfig();
        turretConfig
                .idleMode(IdleMode.kBrake)
                .inverted(Config.turretInverted)
                .smartCurrentLimit(Config.kSmartCurrentLimit)
                .voltageCompensation(Config.kNominalVoltage);
        turretConfig.encoder
                .positionConversionFactor(Physical.turretEncoderPositionFactor)
                .velocityConversionFactor(Physical.turretEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI)
                .pid(Tunings.turretP, 0.0, Tunings.turretD);
        turretConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / Config.kUpdateFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turretSpark,
                5,
                () -> turretSpark.configure(
                        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(turretSpark, 5, () -> turretEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Update turret inputs
        sparkStickyFault = false;
        ifOk(
                turretSpark,
                turretEncoder::getPosition,
                (value) -> inputs.turretPosition = new Rotation2d(value).minus(Rotation2d.kZero));
        ifOk(turretSpark, turretEncoder::getVelocity, (value) -> inputs.turretVelocityRadPerSec = value);
        ifOk(
                turretSpark,
                new DoubleSupplier[] { turretSpark::getAppliedOutput, turretSpark::getBusVoltage },
                (values) -> inputs.turretAppliedVolts = values[0] * values[1]);
        ifOk(turretSpark, turretSpark::getOutputCurrent, (value) -> inputs.turretCurrentAmps = value);
        inputs.turretConnected = turretConnectedDebouncer.calculate(!sparkStickyFault);
    }

    @Override
    public void setTurretOpenLoop(double output) {
        turretSpark.setVoltage(output);
    }

    @Override
    public void setTurretPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.getRadians(), -Math.PI, Math.PI);
        turretController.setSetpoint(setpoint, ControlType.kPosition);
    }

    @Override
    public void setTurretSpeed(double speed) {
        turretSpark.set(speed);
    }
}
