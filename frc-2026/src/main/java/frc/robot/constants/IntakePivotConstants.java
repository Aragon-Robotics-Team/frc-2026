// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakePivotConstants {
    public static final int kMotorID = 1; // The motor ID
    public static final int kEncoderChannel = 1; // The channel to the encoder

    public static final double kUpperLimitTicks = 1000; // The maximum amount of ticks should not go over
    public static final double kStartingLimitTicks = 0; // Minimum ticks should start in this and stay not go below
    public static final double kTicksPerRotation = 4000;

    public static final double kTicksToRads = 1.0 / kTicksPerRotation * 2.0 * Math.PI;

    public static final double kArcadeSpeedMultiplier = 0.5; // The speed multplier for the arcade command

    public static final double kP = 0.35; //0.5;
    public static final double kI = 0.1;
    public static final double kD = 0.0;
    public static final double kPIDIZone = 0.3;
    public static final double kTargetTicks = 100;

    // probably not voltage
    public static final double kS = 0.0; // voltage to overcome static friction/inertia, no friction in sim
    public static final double kG = 0.05; // voltage needed to overcome gravity 
    public static final double kV = 0.05; // voltage needed to overcome friction while in motion, no friction in sim
    public static final double kA = 0.02; // voltage needed to induce acceleration

    public static final double kMaxVelocity = 3.0;
    public static final double kMaxAcceleration = 2.0;

    public static final double kSimStartAngle = 0;
    public static final double kMinAngle = 0.0;
    public static final double kMaxAngle = Math.PI / 2.0;

    public static final double kIntakeLength = Units.inchesToMeters(16.5); // In meters
    public static final double kIntakeMass = Units.lbsToKilograms(6.8); // In kilograms

    public static final double kPivotReduction = 32.4 / 1.0;

    public static final boolean kSimGravity = true;
    public static final double kEncoderNoise = 0.0;
    public static final double kTicksPerPulse = (2.0 * Math.PI) / kTicksPerRotation;
    public static final double kLoopTime = 0.02; // In seconds
}
