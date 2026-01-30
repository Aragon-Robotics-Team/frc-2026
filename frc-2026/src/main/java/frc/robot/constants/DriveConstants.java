// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    // CAN IDs
    public static final int kGyroID = 0;

    // CAN IDs
    public static final int kFrontLeftTurnID = 14;
    public static final int kFrontRightTurnID = 11;
    public static final int kBackLeftTurnID = 12;
    public static final int kBackRightTurnID = 13;

    // CAN IDs
    public static final int kFrontLeftDriveID = 5;
    public static final int kFrontRightDriveID = 1;
    public static final int kBackLeftDriveID = 2;
    public static final int kBackRightDriveID = 3;

    // DIO IDs
    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kFrontRightAbsoluteEncoderPort = 1;
    public static final int kBackLeftAbsoluteEncoderPort = 2;
    public static final int kBackRightAbsoluteEncoderPort = 3;

    public static final double kFrontLeftEncoderOffset = 1.85;
    public static final double kFrontRightEncoderOffset = 3.7;
    public static final double kBackLeftEncoderOffset = 2.52;
    public static final double kBackRightEncoderOffset = 2.81;

    public static final int kFrontLeftDriveReversed = -1;
    public static final int kFrontRightDriveReversed = 1;
    public static final int kBackLeftDriveReversed = -1;
    public static final int kBackRightDriveReversed = 1;

    // Turn PID for each module. Not the whole robot omega PID value. 
    public static final double kPTurn = 0.25;  
    public static final double kITurn = 0;
    public static final double kDTurn = 0;

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kMaxTranslationalMetersPerSecond = 5.8;
    public static final double kGearRatio = 5.36;

    public static final int kLeftYAxisID = 1; // "up" is negative
    public static final int kLeftXAxisID = 0; // "left" is negative 
    public static final int kRightXAxisID = 4; // "left" is negative

    public static final double kSpeedLimiter = 1;

    // Measured to be correct in meters.
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
    new Translation2d(0.276, 0.276),
    new Translation2d(0.276, -0.276),
    new Translation2d(-0.276, 0.276),
    new Translation2d(-0.276, -0.276));
}
