// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(IntakeConstants.kIntakeID);


  /** Creates a new Intake. */
  public Intake() {
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setDutyCycle(double speed) {
    m_motor.set(speed);
  }

  public double getDutyCycle() {
    return m_motor.get();
  }

  public double getRPM() {
    return m_motor.getVelocity().getValueAsDouble();
  }
}
