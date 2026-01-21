// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooters. */
  private TalonFX m_leftMotorCCW = new TalonFX(ShooterConstants.kLeftMotorCCWID);
  private TalonFX m_leftMotorCW = new TalonFX(ShooterConstants.kLeftMotorCWID);

  private TalonFX m_rightMotorCCW = new TalonFX(ShooterConstants.kRightMotorCCWID);
  private TalonFX m_rightMotorCW = new TalonFX(ShooterConstants.kRightMotorCWID);
  

  public Shooter() {
    m_leftMotorCCW.setNeutralMode(NeutralModeValue.Brake);
    m_leftMotorCW.setNeutralMode(NeutralModeValue.Brake);

    m_rightMotorCCW.setNeutralMode(NeutralModeValue.Brake);
    m_rightMotorCW.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setDutyCycle(double DutyCycle) { // between -1.0 and 1.0; positive is shooting outwards
    m_leftMotorCCW.set(DutyCycle); // the motor is spinning CCW so Duty Cycle is negative
    m_leftMotorCW.set(-DutyCycle);  // the motor spinning CW so it is positive

    m_rightMotorCCW.set(DutyCycle); // the motor is spinning CCW so Duty Cycle is negative
    m_rightMotorCW.set(-DutyCycle);  // the motor spinning CW so it is positive
  }

  public static double WheelRPMtoDutyCycle(double WheelRPM) {
    return WheelRPM / ShooterConstants.kWheelMaxRPM;
  }

  public void setRPM(double WheelRPM) {
    setDutyCycle(WheelRPMtoDutyCycle(WheelRPM));
  }

  public double getLeftDutyCycle() {
    return m_leftMotorCW.get(); // we are using the CW motor because this is the one that is spinning positively when shooting
  }

  public double getRightDutyCycle() {
    return m_rightMotorCW.get();
  }

  // double check later
  public double getLeftRPM() {
    return m_leftMotorCCW.getVelocity().getValueAsDouble() * ShooterConstants.kMotorToWheelRatio; // this division operator 
  }

  public double getRightRPM() {
    return m_rightMotorCCW.getVelocity().getValueAsDouble() * ShooterConstants.kMotorToWheelRatio;
  }
 
  public void setVoltage(double voltage) {
    m_leftMotorCCW.setVoltage(-voltage);
    m_leftMotorCW.setVoltage(voltage);

    m_rightMotorCCW.setVoltage(-voltage);
    m_rightMotorCW.setVoltage(voltage);

  }

  public void setIdleRPM() {
    setDutyCycle(WheelRPMtoDutyCycle(ShooterConstants.kIdleRPM));
  }

  public InstantCommand idle() {
    return new InstantCommand(this::setIdleRPM, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
