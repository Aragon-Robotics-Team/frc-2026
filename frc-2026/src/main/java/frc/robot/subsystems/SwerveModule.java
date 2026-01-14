// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private TalonFX m_turnMotor;
  private TalonFX m_driveMotor;
  private DutyCycleEncoder m_absoluteEncoder;
  private PIDController m_pid = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  private double m_absoluteEncoderOffset;
  /** Creates a new SwerveModule. */

  public SwerveModule(int turnMotorID, int driveMotorID, int absoluteEncoderID, double absoluteEncoderOffset) {
    m_turnMotor = new TalonFX(turnMotorID);
    m_driveMotor = new TalonFX(driveMotorID);
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID, 2 * Math.PI, 0);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setModuleState(SwerveModuleState state) {
        Rotation2d m_currentAngle = new Rotation2d((m_absoluteEncoder.get() - m_absoluteEncoderOffset) < 0 ? (m_absoluteEncoder.get() - m_absoluteEncoderOffset + 2 * Math.PI) : (m_absoluteEncoder.get() - m_absoluteEncoderOffset));
        state.optimize(m_currentAngle);
        double m_driveMotorSpeed = state.speedMetersPerSecond / (DriveConstants.kWheelDiameter * Math.PI) * DriveConstants.kMaxMotorSpeed;
        m_driveMotor.set(DriveConstants.kDummySpeedMultiplier);
        double m_currentPosition = m_absoluteEncoder.get() - m_absoluteEncoderOffset;
        if (m_currentPosition < 0) {
            m_currentPosition += 2 * Math.PI;
        }
        if (m_currentPosition > Math.PI) {
            m_currentPosition -= 2 * Math.PI;
        }
        double m_error = m_pid.calculate(m_currentPosition, state.angle.getRadians()) / 5.0;
        m_turnMotor.set(DriveConstants.kDummySpeedMultiplier);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
