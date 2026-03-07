// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMAX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Turret turret;
    private final Shooter shooter;
    private final Intake intake;

    private boolean isFieldRelative = false;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<Double> linearSpeedLimitChooser;
    private final LoggedDashboardChooser<Double> angularSpeedLimitChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOSparkMAX(0),
                        new ModuleIOSparkMAX(1),
                        new ModuleIOSparkMAX(2),
                        new ModuleIOSparkMAX(3));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0));
                turret = new Turret(
                    new TurretIOSparkMAX()
                );
                shooter = new Shooter(
                    new ShooterIOSparkMAX()
                );
                intake = new Intake(
                    new IntakeIOSparkMAX()
                );
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
                turret = new Turret(
                    new TurretIO() {
                    }
                );
                shooter = new Shooter(
                    new ShooterIO() {
                    }
                );
                intake = new Intake(
                    new IntakeIO() {
                    }
                );
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });
                turret = new Turret(
                    new TurretIO() {
                    }
                );
                shooter = new Shooter(
                    new ShooterIO() {
                    }
                );
                intake = new Intake(
                    new IntakeIO() {
                    }
                );
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Set up speed limit chooser
        linearSpeedLimitChooser = new LoggedDashboardChooser<>("Linear Speed Limit");
        angularSpeedLimitChooser = new LoggedDashboardChooser<>("Angular Speed Limit");

        linearSpeedLimitChooser.addDefaultOption("Competition Mode", 1.0);
        linearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        linearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        linearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

        angularSpeedLimitChooser.addDefaultOption("Competition Mode", 1.0);
        angularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
        angularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
        angularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> controller.getLeftY(),
                        () -> controller.getLeftX(),
                        () -> controller.getRightX(),
                        () -> linearSpeedLimitChooser.get(),
                        () -> angularSpeedLimitChooser.get(),
                        () -> {
                            return isFieldRelative;
                        }));

        shooter.setDefaultCommand(
            new RunCommand(
                () -> {
                    shooter.setSpeed(0);
                }, shooter)
        );

        intake.setDefaultCommand(
            new RunCommand(
                () -> {
                    intake.setIntakeSpeed(0);
                    intake.setPositionSpeed(0);
                }, intake)
        );

        turret.setDefaultCommand(
            new RunCommand(
                () -> {
                    turret.setSpeed(0);
                }, turret)
        );

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> controller.getLeftY(),
                                () -> controller.getLeftX(),
                                () -> linearSpeedLimitChooser.get(),
                                () -> angularSpeedLimitChooser.get(),
                                () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                                drive)
                                .ignoringDisable(true));

        // Switch from Field Relative to Robot Relative when Home button is pressed
        controller
                .leftStick()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    isFieldRelative = !isFieldRelative;
                                }));

        controller.y().whileTrue(
                DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> 0.0, // no translation X
                        () -> 0.0, // no translation Y
                        () -> 1.0, // linear speed scale
                        () -> 0.7, // angular speed scale
                        () -> drive.getRotation().plus(vision.getTargetX(0)) // desired heading
                ));

        controller.pov(0).whileTrue(
            new RunCommand(
                () -> {
                    intake.setPositionSpeed(0.3);
                }, intake)
        );

        controller.pov(180).whileTrue(
            new RunCommand(
                () -> {
                    intake.setPositionSpeed(-0.3);
                }, intake)
        );

        controller.pov(90).whileTrue(
            new RunCommand(
                () -> {
                    turret.setSpeed(0.3);
                }, turret)
        );

        controller.pov(270).whileTrue(
            new RunCommand(
                () -> {
                    turret.setSpeed(-0.3);
                }, turret)
        );

        controller.leftBumper().whileTrue(
            new RunCommand(
                () -> {
                    shooter.setSpeed(.7);
                }, shooter)
        );

        controller.rightBumper().whileTrue(
            new RunCommand(
                () -> {
                    intake.setIntakeSpeed(0.7);
                }, intake)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}