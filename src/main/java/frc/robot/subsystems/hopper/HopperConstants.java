// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class HopperConstants {
    public static final class Physical {
        public static final DCMotor kHopperGearbox = DCMotor.getNeo550(1);
        public static final double kHopperMotorReduction = 1.0; // TODO: Change

        // Turret encoder configuration
        public static final double turretEncoderPositionFactor = 2 * Math.PI / kHopperMotorReduction; // Rotor Rotations ->
        // Wheel Radians
        public static final double turretEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kHopperMotorReduction; // Rotor RPM ->
        // Wheel Rad/Sec
    }

    public static final class Config {
        public static final double kUpdateFrequency = 100.0; // Hz

        public static final boolean turretInverted = true;
        public static final int kSmartCurrentLimit = 20; // Less because Neo550
        public static final double kNominalVoltage = 12.0;
    }

    public static final class Ports {
        public static final int HOPPER_PORT_ID = 22;
    }
}
