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

  private SparkMax m_bottomMotorCCW = new SparkMax(ShooterConstants.kSparkMaxID_Bottom_CCW, MotorType.kBrushless);
  private SparkMax m_bottomMotorCW = new SparkMax(ShooterConstants.kSparkMaxID_Bottom_CW, MotorType.kBrushless);
  private SparkMax m_topMotorCW = new SparkMax(ShooterConstants.kSparkMaxID_Top_CW, MotorType.kBrushless);

  private SparkMaxConfig m_config_Bottom_CCW = new SparkMaxConfig();
  private SparkMaxConfig m_config_Bottom_CW = new SparkMaxConfig();
  private SparkMaxConfig m_config_Top = new SparkMaxConfig();

  /** Creates a new Shooter. */
  public Shooter() {
    m_config_Bottom_CCW.idleMode(IdleMode.kCoast);
    m_config_Bottom_CW.idleMode(IdleMode.kCoast);
    m_config_Top.idleMode(IdleMode.kCoast);

    m_bottomMotorCCW.configure(m_config_Bottom_CCW, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomMotorCW.configure(m_config_Bottom_CW, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_topMotorCW.configure(m_config_Top, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //configure motor later
  }

  public void setBottomDutyCycle(double DutyCycle) { // between -1.0 and 1.0; positive is shooting outwards
    m_bottomMotorCCW.set(DutyCycle); // the motor is spinning CCW so Duty Cycle is negative
    m_bottomMotorCW.set(-DutyCycle);  // the motor spinning CW so it is positive
  }

  public void setTopDutyCycle(double DutyCycle) { // between -1.0 and 1.0
    m_topMotorCW.set(DutyCycle); // it should be spinning CW, so positive
  }

  public static double WheelRPMtoDutyCycle(double WheelRPM) {
    return WheelRPM/ShooterConstants.kWheelMaxRPM;
  }

  public void setBottomWheelRPM(double WheelRPM) {
    setBottomDutyCycle(WheelRPMtoDutyCycle(WheelRPM));
  }

  public double getBottomDutyCycle() {
    return m_bottomMotorCW.get(); // we are using the CW motor because this is the one that is spinning positively when shooting
  }

  public double getTopDutyCycle() {
    return m_topMotorCW.get(); // same idea for this one
  }


  public double getBottomRPM() {
    return m_bottomMotorCCW.getEncoder().getVelocity() * ShooterConstants.kBottomMotorToWheelRatio; // this division operator 
  }

  public double getTopRPM() {
    return m_topMotorCW.getEncoder().getVelocity() * ShooterConstants.kTopMotorToWheelRatio;
  }

  public void setBottomVoltage(double voltage) {
    m_bottomMotorCCW.setVoltage(-voltage);
    m_bottomMotorCW.setVoltage(voltage);

  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM at the Wheel", getBottomRPM());
    SmartDashboard.putNumber("Shooter total current", m_bottomMotorCCW.getOutputCurrent() + m_bottomMotorCW.getOutputCurrent());

  }
}
