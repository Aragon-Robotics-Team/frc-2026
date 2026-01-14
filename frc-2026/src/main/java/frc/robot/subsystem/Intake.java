// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax m_motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless); // SparkMax motor in the intake
  private SparkMaxConfig m_config = new SparkMaxConfig(); // configuration required for the motor

  public Intake() {
    m_config.idleMode(IdleMode.kBrake);

    m_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // reset Mode is being removed
  }
  public void setSpeed(double speed){
    m_motor.set(speed);
  }
  public double getSpeed(){
    return m_motor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
