// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants.Physical;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;

  private boolean driveClosedLoop = false;
  private boolean steerClosedLoop = false;
  
  private final PIDController driveController = new PIDController(
    Tunings.driveSimP, 
    0, 
    Tunings.driveSimD
  );
  private final PIDController steerController = new PIDController(
    Tunings.steerSimP, 0, 
    Tunings.steerSimD
  );

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
    Tunings.driveSimKs, 
    Tunings.driveSimKv
  );
  private double driveFF = 0;

  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Create drive and steer sim models
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
          Physical.driveGearbox, 
          0.025, 
          Physical.driveMotorReduction),
        Physical.driveGearbox);
    steerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
          Physical.steerGearbox, 
          0.004, 
          Physical.steerMotorReduction),
        Physical.steerGearbox);

    // Enable wrapping for steer PID
    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFF + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
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

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
    inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad };
    inputs.odometryTurnPositions = new Rotation2d[] { inputs.steerPosition };
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    steerClosedLoop = false;
    steerAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFF = driveFeedforward.calculate(velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    steerClosedLoop = true;
    steerController.setSetpoint(rotation.getRadians());
  }
}