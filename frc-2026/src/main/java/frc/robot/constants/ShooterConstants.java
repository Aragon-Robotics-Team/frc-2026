// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {
    //
    public static final int kSparkMaxID_Bottom_CCW = 20;
    public static final int kSparkMaxID_Bottom_CW = 6;
    public static final int kSparkMaxID_Top_CW = 2;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

    

    public static final double kBottomMotorToWheelRatio = 30.0/18.0; // this is true
    public static final double kTopMotorToWheelRatio = 24.0/13.0; // not sure if this one is true. 

    public static final double kKrakenMaxRPM = 6000; // WCP's number
    public static final double kWheelMaxRPM = kKrakenMaxRPM * kBottomMotorToWheelRatio; 
}
