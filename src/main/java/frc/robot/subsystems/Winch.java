// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
  /** Creates a new winch. */
  private final SparkBase winch;

  public Winch() {
    winch = new SparkMax(10, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Y() {
    winch.set(0.5);
  }

  public void A() {
    winch.set(-0.5);
  }

  public void stop() {
    winch.set(0);
  }
}
