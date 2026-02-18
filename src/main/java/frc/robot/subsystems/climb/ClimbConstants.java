// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/** Add your docs here. */
public class ClimbConstants {
    public static final class Ports{
        public static final int RIGHT_CLIMB_MOTOR_PORT = 41;
        public static final int LEFT_CLIMB_MOTOR_PORT = 67;
    }
    public static final class Config{
        public static final boolean kRightClimbInverted = false;
        public static final boolean kLeftClimbInverted = false;
        public static final double kLeftClimbNominalVoltage = 12.0;
        public static final double kRightClimbNominalVoltage = 12.0;
        public static final int kLeftClimbCurrentLimit = 30;
        public static final int kRightClimbCurrentLimit = 30;
    }
    public static final class Physical {
        public static final MotorType kMotorType = MotorType.kBrushless;

        public static final double kLeftClimbReduction = 1.0;
        public static final double kRightClimbReduction = 1.0;

        
    }
    
}
