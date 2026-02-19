// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;


/** Add your docs here. */
public class DriveConstants {

    public static final double loopPeriodSecs = 0.02;
        
        /**
         * Physical configurations of the motors
         */
        public static final class Config {
            // Configurations for motors
            public static final MotorType kMotorType = MotorType.kBrushless;
            public static final IdleMode kDriveMode = IdleMode.kBrake;
            public static final boolean kAngleInverted = true;
            public static final boolean kDriveInverted = true;

            // Voltage / Currents
            public static final int kMaxDriveCurrent = 30;
            public static final int kMaxAngleCurrent = 30;
            public static final double kDriveNominalVoltage = 12.0;
            public static final double kAngleNominalVoltage = 12.0;

            // Physical Constants / Measurements
            public static final double X_LENGTH = Units.inchesToMeters(27); // Meters
            public static final double Y_LENGTH = Units.inchesToMeters(32);; // Meters
            public static final double kWheelDiameter = 0.0899; // Meters
            public static final double kDriveReduction = 8.14; // MKI4 LV1 8.14:1
            public static final double kAngleReduction = 150.0 / 7.0; // MKI4

            // Conversion / Device Constants
            public final static double kDrivePositionMetersConversionFactor = Math.PI * kWheelDiameter
                    * kDriveReduction;
            public final static double kAnglePositionMetersConversionFactor = 2.0 * Math.PI * kAngleReduction;
        }

        /**
         * Holds tunings for the motor controllers
         */
        public static final class Tunings {
            public static final double driveKp = 0.1;
            public static final double driveKd = 0.0113;
            public static final double driveKs = 0;
            public static final double driveKv = 0;
            
            public static final double turnKp = 11;
            public static final double turnKd = 0;
        }

        /**
         * Offsets for the absolute encoders
         * 
         * @deprecated See PhoenixTuner Zero functionality.
         */
        public static final class Offsets {
            public static final double FL_OFFSET = 0;// 259.687; // 247.499;
            public static final double FR_OFFSET = 0;//201.533 + 180; // 313.417 + 180.0;
            public static final double BL_OFFSET = 0; //190.567; // 7.646;
            public static final double BR_OFFSET = 0; //150.5566 + 180; // 310.957;
        }

        /**
         * CanID Ports for the motors and absolute encoders.
         * 
         */
        public static final class Ports {
            public static final int FL_DRIVE_PORT= 2;
            public static final int FR_DRIVE_PORT = 4;
            public static final int BL_DRIVE_PORT = 8;
            public static final int BR_DRIVE_PORT = 6;

            public static final int FL_ANGLE_PORT = 1;
            public static final int FR_ANGLE_PORT = 3;
            public static final int BL_ANGLE_PORT = 7;
            public static final int BR_ANGLE_PORT = 5;

            public static final int FL_ENCODER_PORT = 12;
            public static final int FR_ENCODER_PORT = 13;
            public static final int BL_ENCODER_PORT = 11;
            public static final int BR_ENCODER_PORT = 10;
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
