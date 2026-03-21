// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class TurretConstants {
    public static final class Physical {
        public static final DCMotor kTurretGearbox = DCMotor.getNeo550(1);
        public static final double kTurretMotorReduction = 3.0 / 1.0 * 200.0 / 10.0;

        // Turret encoder configuration
        public static final double turretEncoderPositionFactor = 2 * Math.PI / kTurretMotorReduction; // Rotor Rotations ->
        // Wheel Radians
        public static final double turretEncoderVelocityFactor = (2 * Math.PI) / 60.0 / kTurretMotorReduction; // Rotor RPM ->
        // Wheel Rad/Sec
    }

    public static final class Config {
        public static final double kUpdateFrequency = 100.0; // Hz

        public static final boolean turretInverted = true;
        public static final int kSmartCurrentLimit = 20; // Less because Neo550
        public static final double kNominalVoltage = 12.0;
    }

    public static final class Ports {
        public static final int TURRET_PORT_ID = 15;

        public static final int TURRET_LEFT_LIMIT_DIO_PORT = 1;
        public static final int TURRET_RIGHT_LIMIT_DIO_PORT = 0;
    }

    public static final class Tunings {
        public static final double turretP = 0.2;
        public static final double turretD = 0.0;
    }
}
