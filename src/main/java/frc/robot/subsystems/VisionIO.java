// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
    public static class VisionInputs {
      public boolean hasTargets = false;
      public int ID = -1;
    }

  public default void updateValues(VisionInputs input) {}
}
