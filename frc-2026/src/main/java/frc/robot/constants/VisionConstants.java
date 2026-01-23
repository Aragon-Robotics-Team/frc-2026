// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final String kCamName = "Arducam_OV9281_USB_Camera";
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), 
        new Rotation3d(0, Math.toRadians(10), 0)
        );

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded
        );
        
    //To Do: check these values (suspiciously copied from photonvision)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
}
