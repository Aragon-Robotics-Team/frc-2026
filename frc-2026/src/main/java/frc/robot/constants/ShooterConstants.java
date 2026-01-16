// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {
    //
    public static final int kSparkMaxID_Bottom_CCW = 19;
    public static final int kSparkMaxID_Bottom_CW = 16;
    public static final int kSparkMaxID_Top_CW = 2;

    public static final double kP = 0.6;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kNeoMaxRPM = 5676.0; // these are based on REV's numbers
    public static final double kWheelMaxRPM = 9460.0; // REV's numbers
    public static final double kNeoTicksPerRotation = 42;

    public static final double kBottomMotorToWheelRatio = 30.0/18.0; // this is true
    public static final double kTopMotorToWheelRatio = 24.0/13.0; // not sure if this one is true. 
}
