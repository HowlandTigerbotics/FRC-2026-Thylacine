// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.robot.subsystems.drive.DriveConstants.Tunings;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;

  private boolean driveClosedLoop = false;
  private boolean steerClosedLoop = false;
  private PIDController driveController = new PIDController(Tunings.driveSimP, 0, Tunings.driveSimD);
  private PIDController steerController = new PIDController(Tunings.steerSimP, 0, Tunings.steerSimD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 
              0.0011, 
              DriveConstants.Config.kDriveReduction
            ),
            DCMotor.getNEO(1));
    steerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1), 
              0.013, 
              DriveConstants.Config.kSteerReduction
            ),
            DCMotor.getNEO(1));

    // Enable wrapping for turn PID
    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (steerClosedLoop) {
      steerAppliedVolts = steerController.calculate(steerSim.getAngularPositionRad());
    } else {
      steerController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    steerSim.setInputVoltage(MathUtil.clamp(steerAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    steerSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update steer inputs
    inputs.steerConnected = true;
    inputs.steerPosition = new Rotation2d(steerSim.getAngularPositionRad());
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.steerAppliedVolts = steerAppliedVolts;
    inputs.steerCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setSteerOpenLoop(double output) {
    steerClosedLoop = false;
    steerAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = Tunings.driveSimKs * Math.signum(velocityRadPerSec) + Tunings.driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setSteerPosition(Rotation2d rotation) {
    steerClosedLoop = true;
    steerController.setSetpoint(rotation.getRadians());
  }
}