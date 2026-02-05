// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithValue;
import frc.robot.commands.WinchStop;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.drive.DriveConstants.ModuleIndex;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GeomUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Pose2d autoDriveTarget =
      new Pose2d(4.0, 2.0, new Rotation2d());

  private PhotonCamera cam;

  // Subsystems
  private Drive drive;
  private Winch winch;

  private DigitalInput limitSwitch = new DigitalInput(0);

  // OI objects
  private XboxController driverController = new XboxController(0);
  private boolean isFieldRelative = true;

  // Choosers
  private final LoggedDashboardChooser<AutoRoutine> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<JoystickMode> joystickModeChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoLinearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoAngularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0);
    
    PortForwarder.add(5800, "photonvision.local", 5800);
    // Instantiate active subsystems
    if (RobotConstants.getMode() != RobotConstants.Mode.REPLAY) {
      switch (RobotConstants.getRobot()) {
        case ROBOT_2025S:
          drive = new Drive(
            new GyroIONavX2(), 
            new ModuleIOSparkMAX(ModuleIndex.FL),
            new ModuleIOSparkMAX(ModuleIndex.FR), 
            new ModuleIOSparkMAX(ModuleIndex.BL),
            new ModuleIOSparkMAX(ModuleIndex.BR)
          );
          break;
        case ROBOT_SIMBOT:
          drive = null;
          break;
        default:
          break;
      }
    }

    // Instantiate missing subsystems
    drive = drive != null ? drive
        : new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {}, new ModuleIO() {});
    winch = new Winch();

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing",
        new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));
    autoChooser.addOption("Move Forward WIP", new AutoRoutine(
      AutoPosition.ORIGIN,
          new DriveWithValue(
            drive, 
            0, 
            0.3, 
            0, 
            isFieldRelative, 
            null, 
            0.7, 
            0.7, 
            0.7).withTimeout(5)
      ));

 



    // Set up choosers
    joystickModeChooser.addDefaultOption("Standard", JoystickMode.Standard);
    joystickModeChooser.addOption("Tank", JoystickMode.Tank);
    demoLinearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 0.7);
    demoLinearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoLinearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoLinearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    demoAngularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 0.7); // TODO do later
    demoAngularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoAngularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoAngularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    // Alert if in tuning mode
    if (RobotConstants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driving controls
    new Trigger(driverController::getStartButton)
        .or(new Trigger(driverController::getBackButton))
        .onTrue(new InstantCommand(() -> {
          isFieldRelative = !isFieldRelative;
          SmartDashboard.putBoolean("Field Relative", isFieldRelative);
        }).ignoringDisable(true));

    

    SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> -driverController.getLeftY(), () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(), () -> !isFieldRelative,
        () -> joystickModeChooser.get(),
        () -> demoLinearSpeedLimitChooser.get(),
        () -> demoAngularSpeedLimitChooser.get(),
        () -> driverController.getRightTriggerAxis()));

    winch.setDefaultCommand(new WinchStop(winch));
    new Trigger(driverController::getAButton).whileTrue(new RunCommand(
      (() -> {winch.A();}), winch
    ));

    new Trigger(driverController::getYButton).whileTrue(new RunCommand(
      () -> { winch.Y(); }, winch
    ));



    // Reset gyro command
    Command resetGyroCommand = new InstantCommand(() -> {
      drive.setPose(autoDriveTarget);
    }, drive).ignoringDisable(true);
    Command rumbleCommand = new StartEndCommand(
        () -> driverController.setRumble(RumbleType.kRightRumble, 0.5),
        () -> driverController.setRumble(RumbleType.kRightRumble, 0.0)) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    }.withTimeout(0.2);
    new Trigger(driverController::getLeftBumper)
        .and(new Trigger(driverController::getRightBumper))
        .onTrue(resetGyroCommand).onTrue(rumbleCommand);

    // Auto drive controls
    // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
    // .whileTrue(new AutoDriveHard(drive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoRoutine routine = autoChooser.get();
    drive.setPose(routine.position.getPose());
    return routine.command;
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN, TARMAC_A, TARMAC_B, TARMAC_C, TARMAC_D, FENDER_A, FENDER_A_REVERSED, FENDER_B, FENDER_B_REVERSED;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        case TARMAC_A:
          return FieldConstants.referenceA
              .transformBy(GeomUtil.transformFromTranslation(-0.5, 0.7));
        case TARMAC_B:
          return FieldConstants.referenceB
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.2));
        case TARMAC_C:
          return FieldConstants.referenceC
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.1));
        case TARMAC_D:
          return FieldConstants.referenceD
              .transformBy(GeomUtil.transformFromTranslation(-0.5, -0.7));
        case FENDER_A:
          return FieldConstants.fenderA
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        case FENDER_A_REVERSED:
          return FieldConstants.fenderA.transformBy(new Transform2d(
              new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
        case FENDER_B:
          return FieldConstants.fenderB
              .transformBy(GeomUtil.transformFromTranslation(0.5, 0.0));
        case FENDER_B_REVERSED:
          return FieldConstants.fenderB.transformBy(new Transform2d(
              new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(180.0)));
        default:
          return new Pose2d();
      }
    }
  }
}
