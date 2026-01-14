// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private SparkMax m_bottomMotor1 = new SparkMax(ShooterConstants.kSparkMaxIDB1, MotorType.kBrushless);
  private SparkMax m_bottomMotor2 = new SparkMax(ShooterConstants.kSparkMaxIDB2, MotorType.kBrushless);
  private SparkMax m_topMotor = new SparkMax(ShooterConstants.kSparkMaxIDT, MotorType.kBrushless);

  private SparkMaxConfig m_configB1 = new SparkMaxConfig();
  private SparkMaxConfig m_configB2 = new SparkMaxConfig();
  private SparkMaxConfig m_configT = new SparkMaxConfig();

  /** Creates a new Shooter. */
  public Shooter() {
    m_configB1.idleMode(IdleMode.kBrake);
    m_configB2.idleMode(IdleMode.kBrake);
    m_configT.idleMode(IdleMode.kBrake);

    m_bottomMotor1.configure(m_configB1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomMotor2.configure(m_configB2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_topMotor.configure(m_configT, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //configure motor later
  }

  public void setBottomSpeed(double speed) {
    m_bottomMotor1.set(speed);
    m_bottomMotor2.set(speed);
  }

  public void setUpperSpeed(double speed) {
    m_topMotor.set(speed);
  }

  public double getBottomSpeed() {
    return m_bottomMotor1.get();
  }

  public double getUpperSpeed() {
    return m_topMotor.get();
  }

  public double getBottomEncoderPosition() {
    return m_bottomMotor1.getEncoder().getPosition();
  }

  public double getTopEncoderPosition() {
    return m_topMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Bottom Motor Speed", getBottomEncoderPosition());
    SmartDashboard.putNumber("Upper Motor Speed", getUpperSpeed());
  }
}
