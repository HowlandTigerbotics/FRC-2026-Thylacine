// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.DriveConstants.ModuleIndex;
import frc.robot.constants.RobotConstants.RobotType;
import frc.robot.util.SparkMaxDerivedVelocityController;

/** Add your docs here. */
public class ModuleIOSparkMAX implements ModuleIO {   
    // Declarations
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final CANcoder absoluteEncoder;
    private final Rotation2d absoluteEncoderOffset;

    private final SparkMaxDerivedVelocityController driveDerivedVelocityController;

    public ModuleIOSparkMAX(ModuleIndex index) {
        if (RobotConstants.getRobot() == RobotType.ROBOT_2025S) {
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

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
            .inverted(DriveConstants.Config.kDriveInverted)
            .smartCurrentLimit(DriveConstants.Config.kMaxDriveCurrent)
            .voltageCompensation(DriveConstants.Config.kDriveNominalVoltage);

        driveMotor.configure(config, null, PersistMode.kPersistParameters);

        config.inverted(DriveConstants.Config.kAngleInverted)
            .smartCurrentLimit(DriveConstants.Config.kMaxAngleCurrent)
            .voltageCompensation(DriveConstants.Config.kAngleNominalVoltage);
        angleMotor.configure(config, null, PersistMode.kPersistParameters);

        driveDerivedVelocityController =
            new SparkMaxDerivedVelocityController(driveMotor);
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPosition(0.0);

        driveMotor.setCANTimeout(0);
        angleMotor.setCANTimeout(0);
    }

    @Override
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

    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }
}
