// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private TalonFX m_motor = new TalonFX(IntakePivotConstants.kMotorID); // creating the objects on the subsystem
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderChannel);
  public IntakePivot() {
    m_motor.setNeutralMode(NeutralModeValue.Brake); // setting mode to brake
  }


  public void setSpeed(double speed){
    if (IntakePivotConstants.kUpperLimitTicks>=getEncoderTicks()&&getEncoderTicks()>=IntakePivotConstants.kStartingLimitTicks){
      m_motor.set(speed);
    }
    else{
      m_motor.set(0);
    }
  } // setting the speed method

  public double getSpeed() {
    return m_motor.get();
  } // getting the speed method

  public double getEncoderTicks() {
    return m_encoder.get();  
  } // getting the ticks

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
