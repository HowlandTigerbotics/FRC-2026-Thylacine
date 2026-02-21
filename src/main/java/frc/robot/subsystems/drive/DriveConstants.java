// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final class Physical {
    // TODO: Check

    /**
     * Measurements of robot dimensions.
     * 
     * trackWdith refers to the distance between the center of a left wheel and
     * right wheel.
     * wheelBase refers to the distance between the center of a front wheel and a
     * back wheel.
     */
    public static final double trackWidthMeters = Units.inchesToMeters(26.5);
    public static final double wheelBaseMeters = Units.inchesToMeters(26.5);
    public static final double driveBaseRadiusMeters = Math.hypot(trackWidthMeters / 2.0, wheelBaseMeters / 2.0);

    public static final Translation2d[] moduleTranslationsMeters = new Translation2d[] {
        new Translation2d(trackWidthMeters / 2.0, wheelBaseMeters / 2.0),
        new Translation2d(trackWidthMeters / 2.0, -wheelBaseMeters / 2.0),
        new Translation2d(-trackWidthMeters / 2.0, wheelBaseMeters / 2.0),
        new Translation2d(-trackWidthMeters / 2.0, -wheelBaseMeters / 2.0)
    };

    public static final double wheelRadiusMeters = Units.inchesToMeters(1.9375); //CORRECT wheel radius

    public static final DCMotor driveGearbox = DCMotor.getNEO(1);
    public static final double driveMotorReduction =  8.14;  //(45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth
    // and 22 spur teeth

    public static final DCMotor steerGearbox = DCMotor.getNEO(1);
    public static final double steerMotorReduction = 150.0 / 7.0; //CORRECT reduction ratio

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Steer encoder configuration
    public static final double steerEncoderPositionFactor = 2 * Math.PI / steerMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double steerEncoderVelocityFactor = (2 * Math.PI) / 60.0 / steerMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
  }

  public static final class Config {
    // Sensor Frequency
    public static final double odometryFrequency = 100.0; // Hz
    public static final double encoderFrequency = 2000.0; // Hz
    public static final double gyroFrequency = 20.0; // Hz

    // Speed Limits
    public static final double maxSpeedMetersPerSec = 4.8;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 30;
    public static final double driveMotorNominalVoltage = 12.0;

    // Steer motor configuration
    public static final boolean steerInverted = false;
    public static final int steerMotorCurrentLimit = 30;
    public static final double steerMotorNominalVoltage = 12.0;

    // Pathplanner Config
    public static final RobotConfig ppConfig = new RobotConfig(
        Physical.robotMassKg,
        Physical.robotMOI,
        new ModuleConfig(
            Physical.wheelRadiusMeters,
            Config.maxSpeedMetersPerSec,
            Physical.wheelCOF,
            Physical.driveGearbox.withReduction(Physical.driveMotorReduction),
            Config.driveMotorCurrentLimit,
            1),
        Physical.moduleTranslationsMeters);
  }

  public static final class Ports {
    // Device CAN IDs
    public static final int PIGEON_CAN_ID_PORT = 14;

    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 3;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 7;
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 5;

    public static final int FRONT_LEFT_STEER_MOTOR_PORT = 4;
    public static final int FRONT_RIGHT_STEER_MOTOR_PORT = 2;
    public static final int BACK_LEFT_STEER_MOTOR_PORT = 8;
    public static final int BACK_RIGHT_STEER_MOTOR_PORT = 6;

    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 12;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 13;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 11;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 10;
  }

  public static final class Tunings {
    // Zeroed rotation values for each module, see setup instructions
    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Steer PID configuration
    public static final double steerKp = 2.0;
    public static final double steerKd = 0.0;
    public static final double steerSimP = 8.0;
    public static final double steerSimD = 0.0;
    public static final double steerPIDMinInput = 0; // Radians
    public static final double steerPIDMaxInput = 2 * Math.PI; // Radians
  }

}