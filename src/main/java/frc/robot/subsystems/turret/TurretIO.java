// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AutoLogIOInputs.PositionMotorInputs;

/** Add your docs here. */
public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs extends PositionMotorInputs {
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(TurretIOInputs inputs) {
    }

    /** Run the turret motor at the specified open loop value. */
    public default void setTurretOpenLoop(double output) {
    }

    /** Run the turret motor at the specified rotation. */
    public default void setTurretPosition(Rotation2d rotation) {
    }

    /** Run the turret motor at the specified speed. */
    public default void setTurretSpeed(double speed) {
    }
}
