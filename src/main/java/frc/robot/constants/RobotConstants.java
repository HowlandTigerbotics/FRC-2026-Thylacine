// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/** Add your docs here. */
public class RobotConstants {
    private static final RobotType robot = RobotType.ROBOT_SIMBOT;
    public static final double loopPeriodSecs = 0.02;
    public static final boolean tuningMode = false;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.",
            AlertType.ERROR);

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
        if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
            invalidRobotAlert.set(true);
            return RobotType.ROBOT_2025S;
        } else {
            return robot;
        }
        } else {
        return robot;
        }
    }

    public static Mode getMode() {
        switch (getRobot()) {
        case ROBOT_2025S:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

        case ROBOT_SIMBOT:
            return Mode.SIM;

        default:
            return Mode.REAL;
        }
    }

    public static final Map<RobotType, String> logFolders =
        Map.of(RobotType.ROBOT_2025S, "/media/sda2");

    public static enum RobotType {
        ROBOT_2025S, ROBOT_SIMBOT
    }

    public static enum Mode {
        REAL, REPLAY, SIM
    }
}
