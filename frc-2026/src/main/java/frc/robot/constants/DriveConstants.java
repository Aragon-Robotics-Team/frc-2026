// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class DriveConstants {
    public static final int kFrontLeftTurnID = 0;
    public static final int kFrontRightTurnID = 0;
    public static final int kBackLeftTurnID = 0;
    public static final int kBackRightTurnID = 0;

    public static final int kFrontLeftDriveID = 0;
    public static final int kFrontRightDriveID = 0;
    public static final int kBackLeftDriveID = 0;
    public static final int kBackRightDriveID = 0;

    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kFrontRightAbsoluteEncoderPort = 0;
    public static final int kBackLeftAbsoluteEncoderPort = 0;
    public static final int kBackRightAbsoluteEncoderPort = 0;

    public static final int kFrontLeftEncoderOffset = 0;
    public static final int kFrontRightEncoderOffset = 0;
    public static final int kBackLeftEncoderOffset = 0;
    public static final int kBackRightEncoderOffset = 0;

    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final double kWheelDiameter = 0;
    public static final double kDummySpeedMultiplier = 0;
    public static final int kGyroID = 0;
    public static final double kMaxMotorSpeed = 5; //tentative

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(new Translation2d(0.31, 0.31),
    new Translation2d(0.31, -0.31),
    new Translation2d(-0.31, 0.31),
    new Translation2d(-0.31, -0.31));
}
