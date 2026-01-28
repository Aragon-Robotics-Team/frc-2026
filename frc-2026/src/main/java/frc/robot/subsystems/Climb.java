// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(ClimbConstants.kClimbMotorID);
  
  /** Creates a new Climb. */
  public Climb() {
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public double getSpeed() {
    return m_motor.get();
  }

  public double getTickPosition() {
    return m_motor.getPosition().getValueAsDouble()*ClimbConstants.kTicksPerRotation;
  }

  public double getRadianPosition() {
    return ticksToRadiansPosition();
  }

  public double ticksToRadiansPosition() {
    return (getTickPosition()/ClimbConstants.kTicksPerRotation) * ClimbConstants.kRadiansPerRotation;
  }

  
}
