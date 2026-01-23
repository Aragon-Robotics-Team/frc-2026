// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooters. */
  private TalonFX m_leftShooterMotorCCW = new TalonFX(ShooterConstants.kLeftMotorCCWID); // Motors for the left shooter
  private TalonFX m_leftShooterMotorCW = new TalonFX(ShooterConstants.kLeftMotorCWID);

  private TalonFX m_rightShooterMotorCCW = new TalonFX(ShooterConstants.kRightMotorCCWID); // Motors for the right shooter
  private TalonFX m_rightShooterMotorCW = new TalonFX(ShooterConstants.kRightMotorCWID);  

  public Shooter() {
    m_leftShooterMotorCCW.setNeutralMode(NeutralModeValue.Coast);
    m_leftShooterMotorCW.setNeutralMode(NeutralModeValue.Coast);

    m_rightShooterMotorCCW.setNeutralMode(NeutralModeValue.Coast);
    m_rightShooterMotorCW.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setDutyCycle(double dutyCycle) { // between -1.0 and 1.0; positive is shooting outwards
    if (Math.abs(dutyCycle) <= 1) {
      m_leftShooterMotorCCW.set(dutyCycle); // the rollers are above the ball and the motor is spinning CCW so Duty Cycle is positive
      m_leftShooterMotorCW.set(-dutyCycle);  // the motor spinning CW so it is negative

      m_rightShooterMotorCCW.set(dutyCycle);
      m_rightShooterMotorCW.set(-dutyCycle);
    }
  }

  public static double wheelRPMtoDutyCycle(double wheelRPM) {
    return wheelRPM / ShooterConstants.kWheelMaxRPM;
  }

  public void setRPM(double wheelRPM) {
    setDutyCycle(wheelRPMtoDutyCycle(wheelRPM));
  }

  public double getLeftDutyCycle() {
    return m_leftShooterMotorCW.get(); // we are using the CW motor because this is the one that is spinning positively when shooting
  }

  public double getRightDutyCycle() {
    return m_rightShooterMotorCW.get();
  }

  // leftShooterRPM used for calculations
  public double getLeftShooterRPM() {
    return m_leftShooterMotorCCW.getVelocity().getValueAsDouble() * ShooterConstants.kWheelToMotorRatio * 60.0; // convert from RPS to RPM
  }

  public double getRightShooterRPM() {
    return m_rightShooterMotorCCW.getVelocity().getValueAsDouble() * ShooterConstants.kWheelToMotorRatio * 60.0;
  }
 
  public double getLeftShooterAccel() {
    return m_leftShooterMotorCCW.getAcceleration().getValueAsDouble() * ShooterConstants.kWheelToMotorRatio * 60.0;
  }

  public double getRightShooterAccel() {
    return m_rightShooterMotorCCW.getAcceleration().getValueAsDouble() * ShooterConstants.kWheelToMotorRatio * 60.0;
  }

  public void setVoltage(double voltage) {
    m_leftShooterMotorCCW.setVoltage(-voltage);
    m_leftShooterMotorCW.setVoltage(voltage);

    m_rightShooterMotorCCW.setVoltage(-voltage);
    m_rightShooterMotorCW.setVoltage(voltage);
  }

  // public void setIdleRPM() {
  //   setDutyCycle(WheelRPMtoDutyCycle(ShooterConstants.kIdleRPM));
  // }

  // public InstantCommand idle() {
  //   return new InstantCommand(this::setIdleRPM, this);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
