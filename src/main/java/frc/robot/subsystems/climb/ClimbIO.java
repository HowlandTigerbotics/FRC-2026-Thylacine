// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/**
 * <p>ModuleIO is the main abstraction class of SwerveModules</p>
 * 
 * @author Gavin Boyts
 * @version 1.0
 * @since 2/18/2026
 */

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        
        public boolean rightMotorConnected = false;
        public boolean leftMotorConnected = false;
        public double rightArmPosition = 0.0;
        public double leftArmPosition = 0.0;
        public double rightArmAppliedVolts = 0.0;
        public double leftArmAppliedVolts = 0.0;


    }

    /**
     * Updates the set of loggable inputs. 
     * 
     * @param inputs inputs to log and update
     */
    public default void updateInputs(ClimbIOInputs inputs) {}

    /** 
     * Run the drive motor at the specified voltage. 
     * 
     * @param double voltage
    */
    public default void setLeftVoltage(double volts) {}

    /** 
     * Run the turn motor at the specified voltage. 
     * 
     * @param double voltage
     */
    public default void setRightVoltage(double volts) {}
}

