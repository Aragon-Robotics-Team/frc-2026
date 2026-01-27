// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */

  private Servo m_actuator = new Servo(ShooterPivotConstants.kServoChannel);

  private Mechanism2d m_mech = new Mechanism2d(5, 5);
  private MechanismRoot2d m_base = m_mech.getRoot("base", 3, 1);
  private MechanismLigament2d m_elevator = m_base.append(new MechanismLigament2d("elevator", ShooterPivotConstants.kElevatorMinimumLength, 90));

  public ShooterPivot() {
    SmartDashboard.putData("Shooter Pivot Sim", m_mech);
  }

  public double getPosition() {
    return m_actuator.get();
  }

  public void setPosition(double position) {
    if(position >= 0.0 && position <= 1.0){
      m_actuator.set(position);
    } else if(position<0){
      m_actuator.set(0);
    } else if(position>=1){
      m_actuator.set(1);
    } else {
      m_actuator.set(m_actuator.get());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_elevator.setLength(ShooterPivotConstants.kElevatorMinimumLength + ShooterPivotConstants.kServoToActuatorLength*getPosition());
  }
}
