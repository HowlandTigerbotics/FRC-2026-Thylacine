// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    public static final class Physical {
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3.5) / 2.0;
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
                new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
                new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
                new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
        };

        public static final MotorType kMotorType = MotorType.kBrushless;

        public static final double X_LENGTH = Units.inchesToMeters(27); // Meters
        public static final double Y_LENGTH = Units.inchesToMeters(32);; // Meters
        public static final double kWheelDiameterMeters = 0.0899; // Meters
        public static final double kDriveReduction = 8.14; // MKI4 LV1 8.14:1
        public static final double kSteerReduction = 150.0 / 7.0; // MKI4

        // Conversion / Device Constants
        public final static double kDrivePositionConversionFactor = 2.0 * Math.PI / kDriveReduction; // Motor rotations
                                                                                                     // -> wheel radians

        public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60.0; // Wheel
                                                                                                           // radians ->
                                                                                                           // rad/sec

        public static final double kSteerPositionConversionFactor = 2.0 * Math.PI; // Rotations -> Radians
        public static final double kSteerVelocityConversionFactor = kSteerPositionConversionFactor / 60.0; // RPM ->
                                                                                                           // Rad/Sec
    
        public static final double robotMassKg = 74.088;
        public static final double robotMOI = 6.883;
        public static final double wheelCOF = 1.2;
        public static final double maxSpeedMetersPerSec = 4.8;
    }

    public static final class Config {
        // Configurations for motors
        public static final IdleMode kDriveMode = IdleMode.kBrake;
        public static final boolean kSteerInverted = true;
        public static final boolean kDriveInverted = true;

        // Voltage / Currents
        public static final int kMaxDriveCurrent = 30;
        public static final int kMaxSteerCurrent = 30;
        public static final double kDriveNominalVoltage = 12.0;
        public static final double kSteerNominalVoltage = 12.0;

        public static final int sensorRefreshRateMs = 20;
        public static final int odometryRefreshRateMs = 50;

        // PathPlanner configuration
        
        public static final RobotConfig ppConfig = new RobotConfig(
                Physical.robotMassKg,
                Physical.robotMOI,
                new ModuleConfig(
                        Physical.kWheelRadiusMeters,
                        Physical.maxSpeedMetersPerSec,
                        Physical.wheelCOF,
                        DCMotor.getNEO(1).withReduction(Physical.kDriveReduction),
                        kMaxDriveCurrent,
                        1),
                Physical.moduleTranslations);
    }

    /**
     * Holds tunings for the motor controllers
     */
    public static final class Tunings {
        public static final double driveP = 0.1;
        public static final double driveD = 0.0;
        public static final double driveKs = 0.12349;
        public static final double driveKv = 0.13477;

        public static final double steerP = 11;
        public static final double steerD = 0;

        // Simulation Tunings
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;

        public static final double steerSimP = 8.0;
        public static final double steerSimD = 0.0;
    }

    /**
     * Offsets for the absolute encoders
     * 
     * @deprecated See PhoenixTuner Zero functionality.
     */
    public static final class Offsets {
        public static final double FL_OFFSET = 0;// 259.687; // 247.499;
        public static final double FR_OFFSET = 0;// 201.533 + 180; // 313.417 + 180.0;
        public static final double BL_OFFSET = 0; // 190.567; // 7.646;
        public static final double BR_OFFSET = 0; // 150.5566 + 180; // 310.957;
    }

    /**
     * CanID Ports for the motors and absolute encoders.
     * 
     */
    public static final class Ports {
        public static final int FL_DRIVE_PORT = 3;
        public static final int FR_DRIVE_PORT = 1;
        public static final int BL_DRIVE_PORT = 7;
        public static final int BR_DRIVE_PORT = 5;

        public static final int FL_STEER_PORT = 4;
        public static final int FR_STEER_PORT = 2;
        public static final int BL_STEER_PORT = 8;
        public static final int BR_STEER_PORT = 6;

        public static final int FL_ENCODER_PORT = 12;
        public static final int FR_ENCODER_PORT = 13;
        public static final int BL_ENCODER_PORT = 11;
        public static final int BR_ENCODER_PORT = 10;

        public static final int PIGEON_CAN_PORT = 14;
    }

    /**
     * Enum values of the physical locations.
     * 
     * FL = Front Left
     * BR = Back Right
     */
    public static enum ModuleIndex {
        FL, FR, BL, BR;
    }
}
