// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.outtake.OuttakeConstants.*;

/**
 * 
 * @author Joshua Siegfried, Kyle Johns
 * @version 1.0
 * @since 2/18/2026
 * 
 */

/** Add your docs here. */
public class  OuttakeIOReal implements OuttakeIO {
    public final SparkMax turretMotor;
    public final SparkMax flywheelMotor;

    public final RelativeEncoder turretEncoder;
    public final RelativeEncoder flywheelEncoder;
    
    public OuttakeIOReal() {
        turretMotor = new SparkMax(Ports.TURRET_MOTOR_PORT, Physical.kMotorType);
        flywheelMotor = new SparkMax(Ports.FLYWHEEL_MOTOR_PORT, Physical.kMotorType);

        turretEncoder = turretMotor.getEncoder();
        flywheelEncoder = flywheelMotor.getEncoder();

        var config = new SparkMaxConfig();

        // Configure Turret Motor
        config.inverted(false)
            .smartCurrentLimit(Config.kTurretCurrentLimit)
            .voltageCompensation(Config.kTurretNominalVoltage);


        turretMotor.configure(config, ResetMode.kResetSafeParameters, null);

        // Configure Flywheel Motor
        config.inverted(Config.kFlywheelInverted)
            .smartCurrentLimit(Config.kFlywheelCurrentLimit)
            .voltageCompensation(Config.kFlywheelNominalVoltage);


        flywheelMotor.configure(config, ResetMode.kResetSafeParameters, null);
    }
    
    @Override
    public void updateInputs(OuttakeIOInputs inputs) {}

    /** 
     * Run the turret (azimuth) motor at the specified voltage. 
     * 
     * @param double voltage
    */
    @Override
    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    /** 
     * Run the flywheel (shooter) motor at the specified voltage. 
     * 
     * @param double voltage
    */
    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelMotor.setVoltage(volts);
    }
}