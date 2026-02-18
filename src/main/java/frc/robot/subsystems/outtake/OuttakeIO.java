// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

/**
 * 
 * @author Joshua Siegfried, Kyle Johns
 * @version 1.0
 * @since 2/18/2026
 */
public interface OuttakeIO {
    @AutoLog
    public static class OuttakeIOInputs {

        public boolean turretConnected = false;
        public double turretPositionRad = 0.0;
        public double turretVelocityRadPerSec = 0.0;
        public double turretAppliedVolts = 0.0;

        public boolean flywheelConnected = false;
        public double flywheelPositionRad = 0.0;
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;
    }

    /**
     * Updates the set of loggable inputs. 
     * 
     * @param inputs inputs to log and update
     */
    public default void updateInputs(OuttakeIOInputs inputs) {}

    /** 
     * Run the turret (azimuth) motor at the specified voltage. 
     * 
     * @param double voltage
    */
    public default void setTurretVoltage(double volts) {}

    /** 
     * Run the flywheel (shooter) motor at the specified voltage. 
     * 
     * @param double voltage
    */
    public default void setFlywheelVoltage(double volts) {}
}
