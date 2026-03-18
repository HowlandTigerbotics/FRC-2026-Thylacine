// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class HopperIOSparkMAX implements HopperIO {
  // Hardware objects
  private final SparkBase hopperSpark;
  private final RelativeEncoder hopperEncoder;

  // Connection debouncers
  private final Debouncer hopperConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  
  public HopperIOSparkMAX() {
    hopperSpark = new SparkMax(Ports.HOPPER_PORT_ID, MotorType.kBrushless);

    hopperEncoder = hopperSpark.getEncoder();

    configureMotors();
  }

  private void configureMotors() {
    // TODO: Update the constants later
    var hopperConfig = new SparkMaxConfig();
    hopperConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0);
    hopperConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    hopperConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0.0, 0);
    hopperConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 20.0)) // TODO Change
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        hopperSpark,
        5,
        () -> hopperSpark.configure(
            hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(hopperSpark, 5, () -> hopperEncoder.setPosition(0.0));

  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Update hopper inputs
    sparkStickyFault = false;
    ifOk(hopperSpark, hopperEncoder::getPosition, (value) -> inputs.hopperPositionRad = value);
    ifOk(hopperSpark, hopperEncoder::getVelocity, (value) -> inputs.hopperVelocityRadPerSec = value);
    ifOk(
        hopperSpark,
        new DoubleSupplier[] { hopperSpark::getAppliedOutput, hopperSpark::getBusVoltage },
        (values) -> inputs.hopperAppliedVolts = values[0] * values[1]);
    ifOk(hopperSpark, hopperSpark::getOutputCurrent, (value) -> inputs.hopperCurrentAmps = value);
    inputs.hopperConnected = hopperConnectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setHopperPercent(double percent) {
    hopperSpark.set(percent);
  }
}
