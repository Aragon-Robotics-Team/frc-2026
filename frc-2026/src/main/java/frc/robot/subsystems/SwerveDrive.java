// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {

  private Canandgyro m_imu = new Canandgyro(DriveConstants.kGyroID);

  private SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftTurnID, DriveConstants.kFrontLeftDriveID, DriveConstants.kFrontLeftAbsoluteEncoderPort, DriveConstants.kFrontLeftEncoderOffset);
  private SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightTurnID, DriveConstants.kFrontRightDriveID, DriveConstants.kFrontRightAbsoluteEncoderPort, DriveConstants.kFrontRightEncoderOffset);
  private SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftTurnID, DriveConstants.kBackLeftDriveID, DriveConstants.kBackLeftAbsoluteEncoderPort, DriveConstants.kBackLeftEncoderOffset);
  private SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightTurnID, DriveConstants.kBackRightDriveID, DriveConstants.kBackRightAbsoluteEncoderPort, DriveConstants.kBackRightEncoderOffset);
  
  public void driveRobot(ChassisSpeeds chassisSpeed) {
      //CREATE CODE TO COMMAND SWERVE MODULES TO MOVE BASED ON CHASSIS SPEEDS
      SwerveModuleState[] moduleStates = DriveConstants.kKinematics.toSwerveModuleStates(chassisSpeed);
      m_frontLeft.setModuleState(moduleStates[0]);
      m_frontRight.setModuleState(moduleStates[1]);
      m_backLeft.setModuleState(moduleStates[2]);
      m_backRight.setModuleState(moduleStates[3]);
      //LOG MODULE STATE SO IT CAN BE DISPLAYED IN ADVANTAGE SCOPE
      Logger.recordOutput("front left", moduleStates[0]);
      Logger.recordOutput("front right", moduleStates[1]);
      Logger.recordOutput("back left", moduleStates[2]);
      Logger.recordOutput("back right", moduleStates[3]);
      Logger.recordOutput("swerve module states", moduleStates);
  }

  public Rotation2d getMeasuredAngle() {
    //RETURN ANGLE FROM IMU
    return m_imu.getRotation2d();
  }
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
