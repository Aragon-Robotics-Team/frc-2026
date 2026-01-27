// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.hardware.DeviceIdentifier;

/** Add your docs here. */
public class IntakePivotConstants {
    public static final int kMotorID = 1; // The motor ID
    public static final int kEncoderChannel = 1; // The channel to the encoder

    public static final double kUpperLimitTicks = 1000; // The maximum amount of ticks should not go over
    public static final double kStartingLimitTicks = -100; // Minimum ticks should start in this and stay not go below
    public static final double kTicksPerRotation = 4000;

    public static final double kArcadeSpeedMultiplier = 0.5; // The speed multplier for the arcade command

    public static final double kP = 10.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kPIDIZone = 5;
    public static final double kTargetTicks = 1000;

    public static final double kS = 0.0; // voltage to overcome static friction/inertia
    public static final double kG = 0.0; // voltage needed to overcome gravity
    public static final double kV = 0.0; // voltage needed to overcome friction while in motion
    public static final double kA = 0.0; // voltage needed to induce acceleration

    public static final double kMaxVelocity = 5;
    public static final double kMaxAcceleration = 0.5;

    public static final double kSimStartAngle = 0; // Math.PI / 2.0; // Angles in radians
    public static final double kMinAngle = 0.0;
    public static final double kMaxAngle = Math.PI / 2.0;

    public static final double kIntakeLength = 0.3; // In meters
    public static final double kIntakeMass = 5.0; // In kilograms

    public static final double kPivotReduction = 5.0;

    public static final boolean kSimGravity = true;
    public static final double kEncoderNoise = 0.0;
    public static final double kTicksPerPulse = (2.0 * Math.PI) / kTicksPerRotation;
    public static final double kLoopTime = 0.02; // In seconds
}
