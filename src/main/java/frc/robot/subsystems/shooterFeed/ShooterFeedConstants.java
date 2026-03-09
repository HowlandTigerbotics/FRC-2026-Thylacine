// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterFeed;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ShooterFeedConstants {
    public static final class Physical {
        public static final DCMotor kFeedGearbox = DCMotor.getNEO(1);
        public static final double kFeedMotorReduction = 1.0;

        // Turret encoder configuration
        public static final double feedEncoderPositionFactor = 2 * Math.PI / kFeedMotorReduction; // Rotor
                                                                                                        // Rotations ->
        // Wheel Radians
        public static final double feedEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kFeedMotorReduction; // Rotor
                                                                                                                 // RPM
                                                                                                                 // ->
        // Wheel Rad/Sec
    }

    public static final class Config {
        public static final double kUpdateFrequency = 100.0; // Hz

        public static final boolean kFeedInverted = false;
        public static final int kSmartCurrentLimit = 30;
        public static final double kNominalVoltage = 12.0;
    }

    public static final class Ports {
        public static final int FEED_PORT_ID = 19;
    }
}
