// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.ModuleIndex;

/**
 * <p>
 * Physical Implementation of a SwerveModule with SparkMax Controllers.
 * </p>
 * 
 * @author Bo Kuang
 * @version 1.0
 * @since 2/5/2026
 */
public class ModuleIOSparkMAX implements ModuleIO {
    // Declarations
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final CANcoder absoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController steerController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> steerPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    /**
     * Initializes a motor controllers based on the which position it is.
     * 
     * @param index Index that corresponds to the physical module position
     */
    public ModuleIOSparkMAX(ModuleIndex index) {
        absoluteEncoderOffset = new Rotation2d(
                switch (index) {
                    case FL -> DriveConstants.Offsets.FL_OFFSET;
                    case FR -> DriveConstants.Offsets.FR_OFFSET;
                    case BL -> DriveConstants.Offsets.BL_OFFSET;
                    case BR -> DriveConstants.Offsets.BR_OFFSET;
                    default -> 0;
                });
        driveMotor = new SparkMax(
                switch (index) {
                    case FL -> DriveConstants.Ports.FL_DRIVE_PORT;
                    case FR -> DriveConstants.Ports.FR_DRIVE_PORT;
                    case BL -> DriveConstants.Ports.BL_DRIVE_PORT;
                    case BR -> DriveConstants.Ports.BR_DRIVE_PORT;
                    default -> 0;
                },
                DriveConstants.Config.kMotorType);
        steerMotor = new SparkMax(
                switch (index) {
                    case FL -> DriveConstants.Ports.FL_STEER_PORT;
                    case FR -> DriveConstants.Ports.FR_STEER_PORT;
                    case BL -> DriveConstants.Ports.BL_STEER_PORT;
                    case BR -> DriveConstants.Ports.BR_STEER_PORT;
                    default -> 0;
                },
                DriveConstants.Config.kMotorType);
        absoluteEncoder = new CANcoder(
                switch (index) {
                    case FL -> DriveConstants.Ports.FL_ENCODER_PORT;
                    case FR -> DriveConstants.Ports.FR_ENCODER_PORT;
                    case BL -> DriveConstants.Ports.BL_ENCODER_PORT;
                    case BR -> DriveConstants.Ports.BR_ENCODER_PORT;
                    default -> 0;
                });

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        driveController = driveMotor.getClosedLoopController();
        steerController = steerMotor.getClosedLoopController();

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
        steerPositionQueue = SparkOdometryThread.getInstance().registerSignal(steerMotor, steerEncoder::getPosition);
    }

    /**
     * Configures the motor controllers and all encoders.
     */
    public void configure() {
        // Configure drive motor
        var driveConfig = new SparkFlexConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.Config.kMaxDriveCurrent)
                .voltageCompensation(DriveConstants.Config.kDriveNominalVoltage);
        driveConfig.encoder
                .positionConversionFactor(DriveConstants.Config.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveConstants.Config.kDriveVelocityConversionFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        DriveConstants.Tunings.driveP,
                        0.0,
                        DriveConstants.Tunings.driveD);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .appliedOutputPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .busVoltagePeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .outputCurrentPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000));
        tryUntilOk(
                driveMotor,
                5,
                () -> driveMotor.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

        // Configure steer motor
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
                .inverted(DriveConstants.Config.kSteerInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.Config.kMaxSteerCurrent)
                .voltageCompensation(DriveConstants.Config.kSteerNominalVoltage);
        steerConfig.encoder
                .positionConversionFactor(DriveConstants.Config.kSteerPositionConversionFactor)
                .velocityConversionFactor(DriveConstants.Config.kSteerVelocityConversionFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        steerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI)
                .pid(
                        DriveConstants.Tunings.steerP,
                        0.0,
                        DriveConstants.Tunings.steerD);
        steerConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .appliedOutputPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .busVoltagePeriodMs((int) (DriveConstants.loopPeriodSecs * 1000))
                .outputCurrentPeriodMs((int) (DriveConstants.loopPeriodSecs * 1000));

        tryUntilOk(
                steerMotor,
                5,
                () -> steerMotor.configure(
                        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Prevent commands from blocking other commands.
        driveMotor.setCANTimeout(0);
        steerMotor.setCANTimeout(0);
    }

    /**
     * Update and logs the inputs
     * 
     * @param inputs
     */
    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveMotor,
                new DoubleSupplier[] { driveMotor::getAppliedOutput, driveMotor::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                steerMotor,
                steerEncoder::getPosition,
                (value) -> inputs.steerPosition = new Rotation2d(value).minus(absoluteEncoderOffset));
        ifOk(steerMotor, steerEncoder::getVelocity, (value) -> inputs.steerVelocityRadPerSec = value);
        ifOk(
                steerMotor,
                new DoubleSupplier[] { steerMotor::getAppliedOutput, steerMotor::getBusVoltage },
                (values) -> inputs.steerAppliedVolts = values[0] * values[1]);
        ifOk(steerMotor, steerMotor::getOutputCurrent, (value) -> inputs.steerCurrentAmps = value);
        inputs.steerConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(absoluteEncoderOffset))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setSteerOpenLoop(double volts) {
        steerMotor.setVoltage(volts);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = DriveConstants.Tunings.driveKs * Math.signum(velocityRadPerSec)
                + DriveConstants.Tunings.driveKv * velocityRadPerSec;
        driveController.setSetpoint(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.plus(absoluteEncoderOffset).getRadians(), -Math.PI, Math.PI);
        steerController.setSetpoint(setpoint, ControlType.kPosition);
    }

}