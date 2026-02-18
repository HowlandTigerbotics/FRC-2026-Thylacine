// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * <p>IntakeIO is the main abstraction class of Intake</p>
 * 
 * @author Bo Kuang
 * @version 1.0
 * @since 2/18/2026
 */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean wristConnected = false;
        public double wristPositionRad = 0.0;
        public double wristVelocityRadPerSec = 0.0;
        public double wristAppliedVolts = 0.0;

        public boolean intakeConnected = false;
        public double intakePositionRad = 0.0;
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVots = 0.0;
    }

    /**
     * Updates the set of loggable inputs. 
     * 
     * @param inputs inputs to log and update
     */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** 
     * Run the wrist motor at the specified voltage. 
     * 
     * @param double voltage
    */
    public default void setWristVoltage(double volts) {}

    /** 
     * Run the intake motor at the specified voltage. 
     * 
     * @param double voltage
     */
    public default void setIntakeVoltage(double volts) {}
}
