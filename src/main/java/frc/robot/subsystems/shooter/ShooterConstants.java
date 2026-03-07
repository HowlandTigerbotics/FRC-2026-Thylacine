// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ShooterConstants {
    public static final class Physical {
        public static final DCMotor kShooterGearbox = DCMotor.getNEO(1);
        public static final double kShooterMotorReduction = 1.0;

        // Turret encoder configuration
        public static final double shooterEncoderPositionFactor = 2 * Math.PI / kShooterMotorReduction; // Rotor
                                                                                                        // Rotations ->
        // Wheel Radians
        public static final double shooterEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kShooterMotorReduction; // Rotor
                                                                                                                 // RPM
                                                                                                                 // ->
        // Wheel Rad/Sec
    }

    public static final class Config {
        public static final double kUpdateFrequency = 100.0; // Hz

        public static final boolean shooterInverted = false;
        public static final int kSmartCurrentLimit = 30; // Less because Neo550
        public static final double kNominalVoltage = 12.0;
    }

    public static final class Ports {
        public static final int FEED_PORT_ID = 19;
        public static final int SHOOTER_PORT_ID = 20;
    }

    public static final class Tunings {
        // Zeroed rotation values for each module, see setup instructions
        // Shooter PID configuration
        public static final double shooterKp = 0.1;
        public static final double shooterKd = 0.0;
        public static final double shooterKs = 0.0;
        public static final double shooterKv = 0.1;
        public static final double shooterSimP = 0.05;
        public static final double shooterSimD = 0.0;
        public static final double shooterSimKs = 0.0;
        public static final double shooterSimKv = 0.0789;
    }
}
