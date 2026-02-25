// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DriveConstants.ModuleIndex;
import frc.robot.util.SparkMaxDerivedVelocityController;

/** 
 * <p> Physical Implementation of a SwerveModule with SparkMax Controllers. </p>
 *  
 * @author Bo Kuang
 * @version 1.0
 * @since 2/5/2026
 */
public class ModuleIOSparkMAX implements ModuleIO {   
    // Declarations
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final CANcoder absoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    private final SparkMaxDerivedVelocityController driveDerivedVelocityController;

    /**
     * Initializes a motor controllers based on the which position it is.
     * 
     * @param index Index that corresponds to the physical module position
     * @throws RuntimeException if RobotState is not physical
     */
    public ModuleIOSparkMAX(ModuleIndex index) {
        if (Constants.currentMode == Mode.REAL) {
            switch (index) {
                case FL:
                    driveMotor = new SparkMax(
                        DriveConstants.Ports.FL_DRIVE_PORT,
                        DriveConstants.Config.kMotorType
                    );
                    angleMotor = new SparkMax(
                        DriveConstants.Ports.FL_ANGLE_PORT, 
                        DriveConstants.Config.kMotorType
                    );
                    absoluteEncoder = new CANcoder(DriveConstants.Ports.FL_ENCODER_PORT);
                    absoluteEncoderOffset = new Rotation2d(DriveConstants.Offsets.FL_OFFSET);
                break;
                case FR:
                    driveMotor = new SparkMax(
                        DriveConstants.Ports.FR_DRIVE_PORT,
                        DriveConstants.Config.kMotorType
                    );
                    angleMotor = new SparkMax(
                        DriveConstants.Ports.FR_ANGLE_PORT, 
                        DriveConstants.Config.kMotorType
                    );
                    absoluteEncoder = new CANcoder(DriveConstants.Ports.FR_ENCODER_PORT);
                    absoluteEncoderOffset = new Rotation2d(DriveConstants.Offsets.FR_OFFSET);
                break;
                case BL:
                    driveMotor = new SparkMax(
                        DriveConstants.Ports.BL_DRIVE_PORT,
                        DriveConstants.Config.kMotorType
                    );
                    angleMotor = new SparkMax(
                        DriveConstants.Ports.BL_ANGLE_PORT, 
                        DriveConstants.Config.kMotorType
                    );
                    absoluteEncoder = new CANcoder(DriveConstants.Ports.BL_ENCODER_PORT);
                    absoluteEncoderOffset = new Rotation2d(DriveConstants.Offsets.BL_OFFSET);
                break;
                case BR:
                    driveMotor = new SparkMax(
                        DriveConstants.Ports.BR_DRIVE_PORT,
                        DriveConstants.Config.kMotorType
                    );
                    angleMotor = new SparkMax(
                        DriveConstants.Ports.BR_ANGLE_PORT, 
                        DriveConstants.Config.kMotorType
                    );
                    absoluteEncoder = new CANcoder(DriveConstants.Ports.BR_ENCODER_PORT);
                    absoluteEncoderOffset = new Rotation2d(DriveConstants.Offsets.BR_OFFSET);
                break;
                default:
                    throw new RuntimeException("Invalid module index for ModuleIOSparkMAX");
            }
        }
        else
            throw new RuntimeException("Invalid robot for ModuleIOSparkMAX");

        driveDerivedVelocityController =
            new SparkMaxDerivedVelocityController(driveMotor);
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
    }

    /**
     * Configures the motor controllers and all encoders.
     */
    public void configure() {
        SparkMaxConfig config = new SparkMaxConfig();

        // To avoid "brown-outs," limit the current and compensate voltages.
        config.smartCurrentLimit(DriveConstants.Config.kMaxDriveCurrent)
              .voltageCompensation(DriveConstants.Config.kDriveNominalVoltage);

        // Set other configurations
        config.idleMode(DriveConstants.Config.kDriveMode)
              .inverted(DriveConstants.Config.kDriveInverted);

        // Configure
        driveMotor.configure(config, null, PersistMode.kPersistParameters);

        // Repeat for steer motor
        config.inverted(DriveConstants.Config.kAngleInverted)
            .smartCurrentLimit(DriveConstants.Config.kMaxAngleCurrent)
            .voltageCompensation(DriveConstants.Config.kAngleNominalVoltage);
        angleMotor.configure(config, null, PersistMode.kPersistParameters);

        // Reset encoder reading of motor
        angleEncoder.setPosition(0.0);

        // Prevent commands from blocking other commands.
        driveMotor.setCANTimeout(0);
        angleMotor.setCANTimeout(0);
    }

    @Override
    /**
     * Update and logs the inputs
     * 
     * @param inputs
     */
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad =
            Units.rotationsToRadians(driveDerivedVelocityController.getPosition())
                / DriveConstants.Config.kDriveReduction;

        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            driveDerivedVelocityController.getVelocity())
            / DriveConstants.Config.kDriveReduction;

        inputs.driveVelocityFilteredRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(
                driveEncoder.getVelocity()) / DriveConstants.Config.kDriveReduction;

        inputs.driveAppliedVolts =
            driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
            
        inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

        inputs.driveTempCelcius =
            new double[] {driveMotor.getMotorTemperature()};

        inputs.turnAbsolutePositionRad =
            new Rotation2d(absoluteEncoder.getPosition().getValue())
                .minus(absoluteEncoderOffset).getRadians();
        inputs.turnPositionRad =
            Units.rotationsToRadians(angleEncoder.getPosition())
                / DriveConstants.Config.kAngleReduction;
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            angleEncoder.getVelocity()) / DriveConstants.Config.kAngleReduction;
        inputs.turnAppliedVolts =
            angleMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.turnCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {angleMotor.getMotorTemperature()};
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }
}
