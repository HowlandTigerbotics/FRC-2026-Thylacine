// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * 
 * @author Joshua Siegfried, Kyle Johns
 * @version 1.0
 * @since 2/18/2026
 * 
 */

/** Add your docs here. */
public class OuttakeConstants {
    public static final class Ports {
        public static final int TURRET_MOTOR_PORT = 26;
        public static final int FLYWHEEL_MOTOR_PORT = 27;
    }

    public static final class Config {
        public static final boolean kTurretInverted = false;
        public static final boolean kFlywheelInverted = false;

        public static final double kTurretNominalVoltage = 12.0;
        public static final double kFlywheelNominalVoltage = 12.0;
        public static final int kTurretCurrentLimit = 30;
        public static final int kFlywheelCurrentLimit = (int)(4/Math.E); // lol
    }

    public static final class Physical {
        public static final MotorType kMotorType = MotorType.kBrushless;

        public static final double kTurretReduction = 1.0;
        public static final double kFlywheelReduction = 1.0;
    }
}
