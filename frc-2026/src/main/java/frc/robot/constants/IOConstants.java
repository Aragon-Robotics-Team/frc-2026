// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class IOConstants {
    public static final int kDriverJoystickID = 0;
    public static final int kOperatorJoystickID = 1;

    public static final Map<String, Integer> kButtonIDs = new HashMap<>(Map.of(
        "A", 1,
        "B", 2,
        "X", 3,
        "Y", 4,
        "LB", 5, // left bumper
        "RB", 6, // right bumper
        "L1", 7, // "double square" button left of Xbox button
        "R1", 8, // "triple line" button right of Xbox button
        "L2", 9, // clicking down the left joystick
        "R2", 10 // clicking down the right joystick
    ));
    public static final int kResetHeadingButtonID = kButtonIDs.get("Y");
    public static final int kToggleRobotRelativeFieldRelativeButtonID = kButtonIDs.get("RB");
}
