// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.Turret;

/** Add your docs here. */
public class TurretCommands {

    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;

    private TurretCommands() {}

    public static Command turretAtAngle(
            Turret turret,
            DoubleSupplier angularSpeedSupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    // Calculate angular speed
                    double omega = angleController.calculate(
                            turret.getRotation().getRadians(), rotationSupplier.get().getRadians());

                    omega *= angularSpeedSupplier.getAsDouble();
                    turret.setTurretOpenLoop(omega);
                },
                turret)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(turret.getRotation().getRadians()));
    }
}
