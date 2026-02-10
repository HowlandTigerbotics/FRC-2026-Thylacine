// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX2
 * 
 * @deprecated Using Pigeon2
 */
public class GyroIONavX2 implements GyroIO {
  private  AHRS gyro;
  private  double[] xyzDps = new double[3];

  public GyroIONavX2() {
    /**switch (RobotConstants.getRobot()) {
      case ROBOT_2025S:
        gyro = new AHRS(NavXComType.kMXP_SPI);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIONavX2");
    }*/
  }

  public void updateInputs(GyroIOInputs inputs) {
    xyzDps[0] = gyro.getRawGyroX();
    xyzDps[1] = gyro.getRawGyroY();
    xyzDps[2] = gyro.getRawGyroZ();
    inputs.position = new Rotation2d(Units.degreesToRadians(gyro.getYaw()));
    inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);

  }
}
