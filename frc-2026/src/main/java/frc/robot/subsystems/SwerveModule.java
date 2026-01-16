// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private TalonFX m_turnMotor;
  private TalonFX m_driveMotor;
  private DutyCycleEncoder m_absoluteEncoder;
  private PIDController m_pid = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  private double m_absoluteEncoderOffset;
  private final VelocityVoltage m_driveVelocityInput = new VelocityVoltage(0);
  private final DutyCycleOut m_turnMotorInput = new DutyCycleOut(0);
  /** Creates a new SwerveModule. */

  public SwerveModule(int turnMotorID, int driveMotorID, int absoluteEncoderID, double absoluteEncoderOffset) {
    m_turnMotor = new TalonFX(turnMotorID);
    m_driveMotor = new TalonFX(driveMotorID);
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID, 2 * Math.PI, 0);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  private int driveMotorMode = 0;{

  if (DriverStation.isTest()) {driveMotorMode = 1;}}

  public void setModuleState(SwerveModuleState state) {
    double m_driveMotorRotationsPerSecond = state.speedMetersPerSecond / (DriveConstants.kWheelDiameter * Math.PI) * DriveConstants.kGearRatio;
    Logger.recordOutput("drive motor speed", m_driveMotorRotationsPerSecond);
    m_driveMotor.setControl(m_driveVelocityInput.withSlot(driveMotorMode).withVelocity(m_driveMotorRotationsPerSecond));

    Rotation2d m_currentAngle = new Rotation2d((m_absoluteEncoder.get() - m_absoluteEncoderOffset) < 0 ? (m_absoluteEncoder.get() - m_absoluteEncoderOffset + 2 * Math.PI) : (m_absoluteEncoder.get() - m_absoluteEncoderOffset));
    state.optimize(m_currentAngle);
    Logger.recordOutput("current angle", m_currentAngle);
    double m_error = m_pid.calculate(m_currentAngle.getRadians(), state.angle.getRadians());
    Logger.recordOutput("turn error", m_error);
    m_turnMotor.setControl(m_turnMotorInput.withOutput(m_error));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
