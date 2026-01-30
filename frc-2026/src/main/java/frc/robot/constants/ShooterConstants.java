// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterConstants {

    public static final int kLeftMotorCCWID = 20;
    public static final int kLeftMotorCWID = 6;

    public static final int kRightMotorCWID = 0;
    public static final int kRightMotorCCWID = 0;

    public static final double kRollerRadius = Units.inchesToMeters(2.0);

    public static final double kWheelToMotorRatio = 30.0 / 18.0;
    public static final double kKrakenX60MaxRPM = 6000.0;
    //public static final double kWheelMaxRPM = kKrakenX60MaxRPM * kWheelToMotorRatio;

    public static final double kArcadeSpeedModifier = 0.025;

    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kTargetRPM = 6000.0; // RPM at the motor 6000 RPM at the wheel.
    public static final double kIdleRPM = 500.0;
    public static final double kMaxAccel = 6000.0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
}
