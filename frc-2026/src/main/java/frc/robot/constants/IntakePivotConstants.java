// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class IntakePivotConstants {
    public static final int kMotorID = 1; // The motor ID
    public static final int kEncoderChannel = 1; // The channel to the encoder

    public static final double kUpperLimitTicks = 1; // The maximum amount of ticks should not go over
    public static final double kStartingLimitTicks = 0; // Minimum ticks should start in this and stay not go below

    public static final double kArcadeSpeedMultiplier = 0.5; // The speed multplier for the arcade command

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kPIDIZone = 0;
    public static final double kTargetTicks = 0;

    public static final double kMaxVelocity = 1;
    public static final double kMaxAcceleration = 0.5;
}
