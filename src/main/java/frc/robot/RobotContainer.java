// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.intakeFeed.IntakeFeed;
import frc.robot.subsystems.intakeFeed.IntakeFeedIO;
import frc.robot.subsystems.intakeFeed.IntakeFeedIOSparkMAX;
import frc.robot.subsystems.intakePosition.IntakePosition;
import frc.robot.subsystems.intakePosition.IntakePositionIO;
import frc.robot.subsystems.intakePosition.IntakePositionIOSparkMAX;
import frc.robot.subsystems.intakeRoller.IntakeRoller;
import frc.robot.subsystems.intakeRoller.IntakeRollerIO;
import frc.robot.subsystems.intakeRoller.IntakeRollerIOSparkMAX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMAX;
import frc.robot.subsystems.shooterFeed.ShooterFeed;
import frc.robot.subsystems.shooterFeed.ShooterFeedIO;
import frc.robot.subsystems.shooterFeed.ShooterFeedIOSparkMAX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSparkMAX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import static frc.robot.subsystems.vision.VisionConstants.piCameraName;
import static frc.robot.subsystems.vision.VisionConstants.robotToPiCamera;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

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

  private final IntakeFeed intakeFeed;
  private final IntakePosition intakePosition;
  private final IntakeRoller intakeRoller;

  private final Shooter shooter;
  private final ShooterFeed shooterFeed;
  private final Turret turret;

  // Vision
  private final Vision vision;  
  private final Vision turretVision;
  private final PhotonCamera intakeCamera = new PhotonCamera("piCameraName"); // TODO CHANGE NAME

  // Booleans for toggles and states
  private boolean isFieldRelative = false;
  private boolean inAuto = true;
  private boolean inAutoAlignment = true;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Double> linearSpeedLimitChooser;
  private final LoggedDashboardChooser<Double> angularSpeedLimitChooser;

  // Tuning Dashboard Inputs
  private final LoggedDashboardChooser<Double> tensSpeedChooser;
  private final LoggedDashboardChooser<Double> onesSpeedChooser;

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
        intakeFeed = new IntakeFeed(
            new IntakeFeedIOSparkMAX());
        intakePosition = new IntakePosition(
            new IntakePositionIOSparkMAX());
        intakeRoller = new IntakeRoller(
            new IntakeRollerIOSparkMAX());
        shooter = new Shooter(
            new ShooterIOSparkMAX());
        shooterFeed = new ShooterFeed(
            new ShooterFeedIOSparkMAX());
        turret = new Turret(
            new TurretIOSparkMAX());
        // TODO: add cameras to vision
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(piCameraName, robotToPiCamera));
        turretVision = new Vision(
          null, new VisionIOPhotonVision("HD_2MP_WEBCAM", Transform3d.kZero));
          
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

        intakeFeed = new IntakeFeed(
            new IntakeFeedIO() {
            });
        intakePosition = new IntakePosition(
            new IntakePositionIO() {
            });
        intakeRoller = new IntakeRoller(
            new IntakeRollerIO() {
            });
        shooter = new Shooter(
            new ShooterIO() {
            });
        shooterFeed = new ShooterFeed(
            new ShooterFeedIO() {
            });
        turret = new Turret(
            new TurretIO() {
            });
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(piCameraName, robotToPiCamera, drive::getPose));
        
        turretVision = new Vision(
          drive::addVisionMeasurement, new VisionIO() {});
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

        intakeFeed = new IntakeFeed(
            new IntakeFeedIO() {
            });
        intakePosition = new IntakePosition(
            new IntakePositionIO() {
            });
        intakeRoller = new IntakeRoller(
            new IntakeRollerIO() {
            });
        shooter = new Shooter(
            new ShooterIO() {
            });
        shooterFeed = new ShooterFeed(
            new ShooterFeedIO() {
            });
        turret = new Turret(
            new TurretIO() {
            });
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        }, new VisionIO() {
        });

        
        turretVision = new Vision(
          drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    
    //motor = new SparkMax(22, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    // Setup Named Commands
    // Important Note: This Shoot command is with named comd sequences
    // Will run when teh trigger is activated.
    NamedCommands.registerCommand("FeedShooter", 
      Commands.run(
        () -> {
          shooterFeed.setFeedPercent(0.6);
          intakeFeed.setFeedPercent(0.6);
        }, shooterFeed, intakeFeed
      ).withTimeout(2)
       .alongWith(
        Commands.print("Feeding")
      )
    );

    NamedCommands.registerCommand("SpinShooter",
        Commands.run(() -> {
          shooter.setPercent(0.6);
        }, shooter)
        .withTimeout(10)
        .alongWith(
          Commands.print("Spinning")
        )
    );

    // Ignore Above. Increase number

    NamedCommands.registerCommand("Spin Up", 
        Commands.runOnce(() -> {
          shooter.setPercent(0.63);
        }, shooter)
    );

    NamedCommands.registerCommand("Feed", 
        Commands.run(
          () -> {
            intakeFeed.setFeedPercent(0.5);
            shooterFeed.setFeedPercent(0.5);
          }, intakeFeed,shooterFeed).withTimeout(5)
    );

    NamedCommands.registerCommand("Stop", 
          Commands.runOnce(
           () -> {
            intakeFeed.stop();
            shooterFeed.stop();
            shooter.stop();
            intakePosition.stop();
            intakeRoller.stop();
           }  
          )
    );

    NamedCommands.registerCommand("Deploy Intake", 
           Commands.run(
            () -> {
              intakePosition.setPositionPercent(0.2);
            }, intakePosition
           ).withTimeout(2)
    );

    NamedCommands.registerCommand("Spin Intake", 
            Commands.runOnce(
              () -> {
                intakeRoller.setIntakePercent(0.7);
              }, intakeRoller)
    );

    // Check Camera Index
    NamedCommands.registerCommand("Turret Aim", 
              TurretCommands.turretAtAngle(turret, () -> 0.7, 
              () -> turret.getRotation().minus(turretVision.getTargetX(0)))
              .withTimeout(1)
    );

    new EventTrigger("Intake Event").whileTrue(
      Commands.run(
        () -> {
          intakeRoller.setIntakePercent(0.7);
        }, intakeRoller)
    );
    
    
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

    // Add PathPlanner Autos here
    autoChooser.addOption("Test Subsystems", new PathPlannerAuto( "Test subsystem"));
    autoChooser.addOption("Left Safe", new PathPlannerAuto("Left Safe"));
    autoChooser.addOption("Right Safe", new PathPlannerAuto("Right Safe"));

    // Set up speed limit chooser
    linearSpeedLimitChooser = new LoggedDashboardChooser<>("Linear Speed Limit");
    angularSpeedLimitChooser = new LoggedDashboardChooser<>("Angular Speed Limit");

    linearSpeedLimitChooser.addDefaultOption("Competition Mode", 1.0);
    linearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    linearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    linearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    angularSpeedLimitChooser.addDefaultOption("Competition Mode", 1.0);
    angularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    angularSpeedLimitChooser.addOption("Mediumer Speed (50%)", 0.5);
    angularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    angularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    tensSpeedChooser = new LoggedDashboardChooser<>("Tens Speed Chooser");
    onesSpeedChooser = new LoggedDashboardChooser<>("Ones Speed Chooser");

    tensSpeedChooser.addDefaultOption("00", 0.0);
    tensSpeedChooser.addOption("10", 0.1);
    tensSpeedChooser.addOption("20", 0.2);
    tensSpeedChooser.addOption("30", 0.3);
    tensSpeedChooser.addOption("40", 0.4);
    tensSpeedChooser.addOption("50", 0.5);
    tensSpeedChooser.addOption("60", 0.6);
    tensSpeedChooser.addOption("70", 0.7);
    tensSpeedChooser.addOption("80", 0.8);
    tensSpeedChooser.addOption("90", 0.9);
    tensSpeedChooser.addOption("100", 1.0);

    onesSpeedChooser.addDefaultOption("0", 0.00);
    onesSpeedChooser.addOption("1", 0.01);
    onesSpeedChooser.addOption("2", 0.02);
    onesSpeedChooser.addOption("3", 0.03);
    onesSpeedChooser.addOption("4", 0.04);
    onesSpeedChooser.addOption("5", 0.05);
    onesSpeedChooser.addOption("6", 0.06);
    onesSpeedChooser.addOption("7", 0.07);
    onesSpeedChooser.addOption("8", 0.08);
    onesSpeedChooser.addOption("9", 0.09);

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> linearSpeedLimitChooser.get(),
            () -> angularSpeedLimitChooser.get(),
            () -> {
              return isFieldRelative;
            }));

    intakeFeed.setDefaultCommand(
        new RunCommand(
            () -> {
              intakeFeed.stop();
            }, intakeFeed));

    intakePosition.setDefaultCommand(
        new RunCommand(
            () -> {
              intakePosition.stop();
            }, intakePosition));

    intakeRoller.setDefaultCommand(
        new RunCommand(
            () -> {
              if (!inAuto)
                if (intakeCamera.getLatestResult().hasTargets()) intakeRoller.setIntakePercent(0.7);
                else intakeRoller.stop();
            }, intakeRoller));

    shooter.setDefaultCommand(
        new RunCommand(
            () -> {
              if (!inAuto)
                shooter.stop();
            }, shooter));

    shooterFeed.setDefaultCommand(
        new RunCommand(
            () -> {
              shooterFeed.stop();
            }, shooterFeed));

    turret.setDefaultCommand(
        TurretCommands.turretAtAngle(turret, () -> 0.7, () -> turretVision.getTargetX(0).minus(turret.getRotation())));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .b()
        .whileTrue(
          
            Commands.run(
              () -> {
                shooter.setPercent(0.63);
              }, shooter).alongWith(
                Commands.waitSeconds(0.5)
                .andThen(
                  
                    Commands.run(
                      () -> {
                        intakeFeed.setFeedPercent(0.7);
                        shooterFeed.setFeedPercent(0.7);
                      }, intakeFeed, shooterFeed)
                  
                )
              )  
          
        );

    // Switch from Field Relative to Robot Relative when Home button is pressed
    controller
        .leftStick()
        .onTrue(
            Commands.runOnce(
                () -> {
                  isFieldRelative = !isFieldRelative;
                }));
    
    controller.rightStick()
        .onTrue(
          Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
                .ignoringDisable(true));

    controller.y().whileTrue(
        TurretCommands.turretAtAngle(turret, () -> 0.7, () -> turret.getRotation().minus(turretVision.getTargetX(0))));

    controller.pov(0).whileTrue(
        new RunCommand(
            () -> {
              intakePosition.setPositionPercent(0.2);
            }, intakePosition));

    controller.pov(180).whileTrue(
        new RunCommand(
            () -> {
              intakePosition.setPositionPercent(-0.2);
            }, intakePosition));

    controller.pov(90).whileTrue(
        new RunCommand(
            () -> {
              turret.setTurretPercent(-0.05);
            }, turret));

    controller.pov(270).whileTrue(
        new RunCommand(
            () -> {
              turret.setTurretPercent(0.05);
            }, turret));

    controller.leftBumper().whileTrue(
        new RunCommand(
            () -> {
              intakeRoller.setIntakePercent(0.7);
            }, intakeRoller));

    controller.rightBumper().whileTrue(
        new RunCommand(
            () -> {
              shooter.setPercent(0.63);
            }, shooter));

    controller.leftTrigger(0.3).whileTrue(
      new RunCommand(
        () -> {
          intakeFeed.setFeedPercent(.7);
        }, intakeFeed)
    );

    controller.rightTrigger(0.3).whileTrue(
      new RunCommand(
        () -> {
          shooterFeed.setFeedPercent(0.7);
        }, shooterFeed)
    );

    controller.start().whileTrue(
        TurretCommands.turretAtAngle(
          turret, 
          () -> 0.7,
          () -> turret.getRotation().minus(vision.getTargetX(0)))
    );

    // TODO: TEST
    controller.back().whileTrue(
      new RunCommand(
        () -> {
          double k = tensSpeedChooser.get() + onesSpeedChooser.get(); // Increase to reduce speed further
          double percent = 0.63 + k * Math.log(-31.05909 * Math.log(turretVision.getTargetAreaPercent(0) / 100.0) - 72.3987);
          if (percent > 1 || percent < 0) {
            SmartDashboard.putString("Auto Speed", "TOO FAR OR TOO CLOSE");
            return;
          }
          
            SmartDashboard.putString("Auto Speed", "GOOD");
          shooter.setPercent(percent);
        }, shooter)
    );

    controller.a().whileTrue(
      new RunCommand(
        () -> {
          intakeFeed.setFeedPercent(-0.3);
        }, intakeFeed
      )
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

  public void setInAuto(boolean inAuto) {
    this.inAuto = inAuto;
  }
}