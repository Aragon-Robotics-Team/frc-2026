// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class DriveConstants {
    public static final int kFrontLeftTurnID = 1;
    public static final int kFrontRightTurnID = 13;
    public static final int kBackLeftTurnID = 2;
    public static final int kBackRightTurnID = 3;

    public static final int kFrontLeftDriveID = 11;
    public static final int kFrontRightDriveID = 14;
    public static final int kBackLeftDriveID = 12;
    public static final int kBackRightDriveID = 52;

    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kFrontRightAbsoluteEncoderPort = 0;
    public static final int kBackLeftAbsoluteEncoderPort = 0;
    public static final int kBackRightAbsoluteEncoderPort = 0;

    public static final int kFrontLeftEncoderOffset = 0;
    public static final int kFrontRightEncoderOffset = 0;
    public static final int kBackLeftEncoderOffset = 0;
    public static final int kBackRightEncoderOffset = 0;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kWheelDiameter = 0;
    public static final int kGyroID = 0;
    public static final double kMaxMotorSpeed = 5; //tentative
    public static final int kGearRatio = 6; //tentative

    public static final int kLeftYAxisID = 0;
    public static final int kLeftXAxisID = 0;
    public static final int kRightXAxisID = 0;

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(new Translation2d(0.31, 0.31),
    new Translation2d(0.31, -0.31),
    new Translation2d(-0.31, 0.31),
    new Translation2d(-0.31, -0.31));
}
