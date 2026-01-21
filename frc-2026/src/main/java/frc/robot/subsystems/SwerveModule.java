// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
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
  private int m_driveReversed;
  private final VelocityVoltage m_driveVelocityInput = new VelocityVoltage(0);
  private final DutyCycleOut m_turnMotorInput = new DutyCycleOut(0);
  private String m_prefix;
  /** Creates a new SwerveModule. */

  public SwerveModule(int turnMotorID, int driveMotorID, int absoluteEncoderID, double absoluteEncoderOffset, int driveReversed, String location) {
    m_turnMotor = new TalonFX(turnMotorID);
    m_driveMotor = new TalonFX(driveMotorID);
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID, 2 * Math.PI, 0);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_driveReversed = driveReversed;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
    m_prefix = location;

    this.initDrivePID(m_driveMotor);
  }
  
  private int driveMotorMode = 0;{

  if (DriverStation.isTest()) {driveMotorMode = 1;}}

  private void initDrivePID(TalonFX driveCTalonFX) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    driveCTalonFX.getConfigurator().apply(slot0Configs);
  }

  public void setModuleState(SwerveModuleState state) {
    double m_currentAngle = m_absoluteEncoder.get() - m_absoluteEncoderOffset; // radians
    state.optimize(new Rotation2d(m_currentAngle));
    double m_driveMotorRotationsPerSecond = state.speedMetersPerSecond / (DriveConstants.kWheelDiameter * Math.PI) * DriveConstants.kGearRatio;
    Logger.recordOutput("drive motor speed", m_driveMotorRotationsPerSecond);
    m_driveMotor.setControl(m_driveVelocityInput.withSlot(driveMotorMode).withVelocity(m_driveMotorRotationsPerSecond));
    
    Logger.recordOutput(m_prefix + "current angle", m_currentAngle);
    double m_error = m_pid.calculate(MathUtil.angleModulus(m_currentAngle), state.angle.getRadians());
    Logger.recordOutput(m_prefix + "turn error remaining", m_error);
    m_turnMotor.setControl(m_turnMotorInput.withOutput(m_error));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(m_prefix, m_absoluteEncoder.get());
  }
}
