// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intakePosition;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeRollerPosition {
  public static final class Physical {
    public static final DCMotor intakeGearbox = DCMotor.getNEO(1);
    public static final double intakeMotorReduction = 4.0 / 1.0 * 32.0 / 24.0 * 24.0 / 32.0;
    // and 22 spur teeth

    public static final DCMotor positionGearbox = DCMotor.getNEO(1);
    public static final double positionMotorReduction = 5.0 / 1.0 * 5.0 / 1.0 * 15.0 / 15.0 * 60.0 / 15.0; 

    // Intake encoder configuration
    public static final double intakeEncoderPositionFactor = 2 * Math.PI / intakeMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double intakeEncoderVelocityFactor = (2 * Math.PI) / 60.0 / intakeMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Position encoder configuration
    public static final double positionEncoderPositionFactor = 2 * Math.PI / positionMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double positionEncoderVelocityFactor = (2 * Math.PI) / 60.0 / positionMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec
  }

  public static final class Config {
    // Sensor Frequency
    public static final double odometryFrequency = 100.0; // Hz
    public static final double encoderFrequency = 2000.0; // Hz

    // Intake motor configuration
    public static final int intakeMotorCurrentLimit = 30;
    public static final double intakeMotorNominalVoltage = 12.0;

    // Position motor configuration
    public static final boolean positionInverted = true;
    public static final int positionMotorCurrentLimit = 30;
    public static final double positionMotorNominalVoltage = 12.0;
  }

  public static final class Ports {
    // Device CAN ID
    public static final int POSITION_MOTOR_PORT = 19;
  }

  public static final class Tunings {
    // Zeroed rotation values for each module, see setup instructions
    // Intake PID configuration
    public static final double intakeKp = 0.0;
    public static final double intakeKd = 0.0;
    public static final double intakeKs = 0.0;
    public static final double intakeKv = 0.1;
    public static final double intakeSimP = 0.05;
    public static final double intakeSimD = 0.0;
    public static final double intakeSimKs = 0.0;
    public static final double intakeSimKv = 0.0789;

    // Position PID configuration
    public static final double positionKp = 2.0;
    public static final double positionKd = 0;
    public static final double positionSimP = 8.0;
    public static final double positionSimD = 0.0;
    public static final double positionPIDMinInput = 0; // Radians
    public static final double positionPIDMaxInput = 2 * Math.PI; // Radians
  }

}