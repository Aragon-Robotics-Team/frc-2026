// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    public static final int kFrontLeftTurnID = 11;
    public static final int kFrontRightTurnID = 14;
    public static final int kBackLeftTurnID = 12;
    public static final int kBackRightTurnID = 13;

    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontRightDriveID = 5;
    public static final int kBackLeftDriveID = 2;
    public static final int kBackRightDriveID = 3;

    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kFrontRightAbsoluteEncoderPort = 1;
    public static final int kBackLeftAbsoluteEncoderPort = 2;
    public static final int kBackRightAbsoluteEncoderPort = 3;

    public static final double kFrontLeftEncoderOffset = 5.26;
    public static final double kFrontRightEncoderOffset = 0.23 + Math.PI;
    public static final double kBackLeftEncoderOffset = 2.35;
    public static final double kBackRightEncoderOffset = 2.82 + Math.PI;

    public static final int kFrontLeftDriveReversed = 1;
    public static final int kFrontRightDriveReversed = -1;
    public static final int kBackLeftDriveReversed = -1;
    public static final int kBackRightDriveReversed = 1;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final int kGyroID = 0;
    public static final double kMaxMotorSpeed = 5; //tentative
    public static final int kGearRatio = 6; //tentative

    public static final int kLeftYAxisID = 1;
    public static final int kLeftXAxisID = 0;
    public static final int kRightXAxisID = 4;

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
    new Translation2d(0.295, 0.292),
    new Translation2d(0.295, -0.292),
    new Translation2d(-0.295, 0.292),
    new Translation2d(-0.295, -0.292));
}
