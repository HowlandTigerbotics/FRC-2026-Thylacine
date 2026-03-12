// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutoLogIOInputs {
    public static class PositionMotorInputs {
        public boolean positionMotorConnected = false;
        public Rotation2d positionMotorPosition = Rotation2d.kZero;
        public double positionMotorVelocityRadPerSec = 0.0;
        public double positionMotorAppliedVolts = 0.0;
        public double positionMotorCurrentAmps = 0.0;
    }

    public static class VelocityMotorInputs {
        public boolean velocityMotorConnected = false;
        public double velocityMotorPositionRad = 0.0;
        public double velocityMotorVelocityRadPerSec = 0.0;
        public double velocityMotorAppliedVolts = 0.0;
        public double velocityMotorCurrentAmps = 0.0;
    }
}
