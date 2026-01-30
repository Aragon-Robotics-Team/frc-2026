// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterPivotConstants {

    public static final int kServoChannel = 0;

    public static final double kArcadePositionMultiplier = 0.2; // I have no idea if this is needed

    public static final double kToPositionTargetHeight = 0.7;

    public static final double kElevatorMinimumLength = 2.3;
    public static final double kServoToActuatorLength = 2;
    public static final double kDriveTrainStart = 4;
    public static final double kDriveTrainHeight = 3;
    public static final double kDriveTrainLength = 5;
    public static final double kDriveTrainWidth = 7;
    public static final double kPivotFromBase = 5;
    public static final double kPivotFromDriveTrain = 5;
    public static final double kPivotX = kDriveTrainStart + 2;
    public static final double kPivotY = kDriveTrainHeight + kPivotFromDriveTrain;
    public static final double kBaseX = kDriveTrainStart + 2;
    public static final double kStartShooterDegrees = 210;
    public static final double kShooterRadius = 3;
    public static final double kShooterVoid = 2;
    public static final int kShooterSize = 17;
}
