// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import frc.robot.subsystems.climb.ClimbConstants.*;

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO{
    public final SparkMax rightClimbMotor;
    public final SparkMax leftClimbMotor;

    public final RelativeEncoder rightClimbEncoder;
    public final RelativeEncoder leftClimbEncoder;

    public ClimbIOReal(){
        rightClimbMotor = new SparkMax(Ports.RIGHT_CLIMB_MOTOR_PORT, Physical.kMotorType);
        leftClimbMotor = new SparkMax(Ports.LEFT_CLIMB_MOTOR_PORT, Physical.kMotorType);

        rightClimbEncoder = rightClimbMotor.getEncoder();
        leftClimbEncoder = rightClimbMotor.getEncoder();

        var config = new SparkMaxConfig();
        
        //configure right motor
        config.inverted(Config.kRightClimbInverted)
            .smartCurrentLimit(Config.kRightClimbCurrentLimit)
            .voltageCompensation (Config.kRightClimbNominalVoltage);
        rightClimbMotor.configure(config, ResetMode.kResetSafeParameters, null);
        //configure left motor
        config.inverted(Config.kLeftClimbInverted)
            .smartCurrentLimit(Config.kLeftClimbCurrentLimit)
            .voltageCompensation (Config.kLeftClimbNominalVoltage);
        rightClimbMotor.configure(config, ResetMode.kResetSafeParameters, null);
    }
    @Override
    public  void updateInputs(ClimbIOInputs inputs) {}

    /** 
     * Run the drive motor at the specified voltage. 
     * 
     * @param double voltage
    */
    @Override
    public  void setLeftVoltage(double volts) {
        leftClimbMotor.setVoltage(volts);
    }

    /** 
     * Run the turn motor at the specified voltage. 
     * 
     * @param double voltage
     */
    @Override
    public  void setRightVoltage(double volts) {
        rightClimbMotor.setVoltage(volts);
    }
}

