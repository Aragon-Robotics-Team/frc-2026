// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  SparkMax m_motor = new SparkMax(ClimbConstants.kMotorID, MotorType.kBrushless);
  SparkMaxConfig m_config = new SparkMaxConfig();

  public Climb() {
    m_config.idleMode(IdleMode.kBrake);
    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    if ((speed < 0 && getTicks() <= ClimbConstants.kLowerLimitTicks) || (speed > 0 && getTicks() >= ClimbConstants.kUpperLimitTicks)) {
      speed = 0;
    }
    m_motor.set(speed);
  }

  public double getSpeed() {
    return m_motor.get();
  }

  public double getTicks() {
    return m_motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
