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
    public static final String kLeftCamName = "Arducam_OV9281_USB_Camera";
    public static final String kRightCamName = "Arducam_OV9281_USB_Camera (1)";
    
    public static final Transform3d kLeftRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(5), Units.inchesToMeters(37.5)), 
        new Rotation3d(Math.toRadians(0.25), Math.toRadians(-12.05), 0)
    );

    public static final Transform3d kRightRobotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-5), Units.inchesToMeters(36)), 
        new Rotation3d(Math.toRadians(0.25), Math.toRadians(-16.55), 0)
    );

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltAndymark
    );
        
    //To Do: check these values (suspiciously copied from photonvision)
    public static final Matrix<N3, N1> kLeftSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kLeftMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
    
    public static final Matrix<N3, N1> kRightSingleTagStdDevs = VecBuilder.fill(4,4,8);
    public static final Matrix<N3, N1> kRightMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
}
