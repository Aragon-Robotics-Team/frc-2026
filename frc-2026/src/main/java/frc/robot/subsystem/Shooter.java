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

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private TalonFX m_bottomMotorCCW = new TalonFX(ShooterConstants.kSparkMaxID_Bottom_CCW);
  private TalonFX m_bottomMotorCW = new TalonFX(ShooterConstants.kSparkMaxID_Bottom_CW);

  private StatusSignal<AngularVelocity> m_motorVelocityCCW = m_bottomMotorCCW.getVelocity();
  private StatusSignal<AngularVelocity> m_motorVelocityCW = m_bottomMotorCW.getVelocity();

  private TalonFXConfiguration m_config_bottom_CCW = new TalonFXConfiguration();
  private TalonFXConfiguration m_config_bottom_CW = new TalonFXConfiguration();
  

  /** Creates a new Shooter. */
  public Shooter() {

  }

  public void setBottomDutyCycle(double DutyCycle) { 
    m_bottomMotorCCW.set(DutyCycle);
    m_bottomMotorCW.set(-DutyCycle); 
  }

  public static double WheelRPMtoDutyCycle(double WheelRPM) {
    return WheelRPM/ShooterConstants.kWheelMaxRPM;
  }

  public void setBottomWheelRPM(double WheelRPM) {
    setBottomDutyCycle(WheelRPMtoDutyCycle(WheelRPM));
    System.out.println("Duty cycle commanded: " + WheelRPMtoDutyCycle(WheelRPM));
  }

  public double getBottomDutyCycle() {
    return m_bottomMotorCW.get(); // we are using the CW motor because this is the one that is spinning positively when shooting
  }


  public double getBottomRPM() {

    System.out.println(m_motorVelocityCW.refresh().getValueAsDouble()*60);
    System.out.println(m_motorVelocityCCW.refresh().getValueAsDouble()*60);
    System.out.println("*******");
    return (m_motorVelocityCCW.refresh().getValueAsDouble()*60); 
  }


  // public void setBottomVoltage(double voltage) {
  //   m_bottomMotorCCW.setVoltage(-voltage);
  //   m_bottomMotorCW.setVoltage(voltage);

  // }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM at the Wheel", getBottomRPM());

  }
}
