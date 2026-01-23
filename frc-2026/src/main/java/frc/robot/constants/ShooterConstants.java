// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {

    public static final int kLeftMotorCCWID = 0;
    public static final int kLeftMotorCWID = 1;

    public static final int kRightMotorCWID = 2;
    public static final int kRightMotorCCWID = 3;

    public static final double kWheelToMotorRatio = 30.0 / 18.0;
    public static final double kKrakenX60MaxRPM = 6000.0;
    public static final double kWheelMaxRPM = kKrakenX60MaxRPM * kWheelToMotorRatio;

    public static final double kArcadeSpeedModifier = 0.025;

    public static final double kP = 0.55;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kTargetRPM = 6000.0;
    public static final double kIdleRPM = 500.0;
    public static final double kMaxVel = 12000.0;
    public static final double kMaxAccel = 6000.0;
}
