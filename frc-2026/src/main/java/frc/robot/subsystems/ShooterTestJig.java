// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterTestJigConstants;

public class ShooterTestJig extends SubsystemBase {

  private SparkMax m_main1 = new SparkMax(ShooterTestJigConstants.kMain1ID, MotorType.kBrushless);
  private SparkMax m_main2 = new SparkMax(ShooterTestJigConstants.kMain2ID, MotorType.kBrushless);
  private SparkMax m_upper = new SparkMax(ShooterTestJigConstants.kUpperID, MotorType.kBrushless);

  private SparkMaxConfig m_config = new SparkMaxConfig();

  /** Creates a new Shooter. */
  public ShooterTestJig() {
    m_config.idleMode(IdleMode.kBrake);
    m_main1.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_main2.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_upper.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMainSpeed(double speed) {
    m_main1.set(speed);
    m_main2.set(speed);
  }

  public void setUpperSpeed(double speed) {
    m_upper.set(speed);
  }

  public double getRPM() {
    return m_main1.getEncoder().getVelocity()/ShooterTestJigConstants.kMainGearRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getRPM());
  }
}
