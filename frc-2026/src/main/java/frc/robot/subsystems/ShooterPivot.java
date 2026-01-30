// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterPivotConstants;

public class ShooterPivot extends SubsystemBase {
  /** Creates a new ShooterPivot. */

  private Servo m_actuator = new Servo(ShooterPivotConstants.kServoChannel);

  private Mechanism2d m_mech = new Mechanism2d(7, 8);
  private MechanismRoot2d m_base;
  private MechanismRoot2d m_pivot;
  private MechanismRoot2d m_drive;
  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_drivetrain;
  private MechanismLigament2d[] m_shooter = new MechanismLigament2d[17];
  private MechanismLigament2d[] m_shooterVoid = new MechanismLigament2d[17];

  public ShooterPivot(Mechanism2d mech) {
    m_mech = mech;
    m_base = m_mech.getRoot("base", ShooterPivotConstants.kBaseX, ShooterPivotConstants.kDriveTrainHeight);
    m_drive = m_mech.getRoot("drive", ShooterPivotConstants.kDriveTrainStart, ShooterPivotConstants.kDriveTrainHeight);
    m_pivot = m_mech.getRoot("pivot", ShooterPivotConstants.kPivotX, ShooterPivotConstants.kPivotY);
    m_elevator =  m_base.append(new MechanismLigament2d("elevator", ShooterPivotConstants.kElevatorMinimumLength, 90));
    m_drivetrain = m_drive.append(new MechanismLigament2d("drivetrain", ShooterPivotConstants.kDriveTrainLength, 0, ShooterPivotConstants.kDriveTrainWidth, new Color8Bit(Color.kAntiqueWhite)));

    for (int i = 0; i < ShooterPivotConstants.kShooterSize; i++) {
      m_shooter[i] = m_pivot.append(new MechanismLigament2d("a" + i, ShooterPivotConstants.kShooterRadius, ShooterPivotConstants.kStartShooterDegrees + i * 5, 5, new Color8Bit(Color.kAquamarine)));
      m_shooterVoid[i] = m_pivot.append(new MechanismLigament2d("z" + i, ShooterPivotConstants.kShooterVoid, ShooterPivotConstants.kStartShooterDegrees + i * 5, 5, new Color8Bit(10,20,30)));
    }
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
    //This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    m_elevator.setLength(ShooterPivotConstants.kElevatorMinimumLength + ShooterPivotConstants.kServoToActuatorLength * getPosition());
    m_elevator.setAngle(90 - (Math.acos((m_elevator.getLength() * m_elevator.getLength() + ShooterPivotConstants.kPivotFromBase * ShooterPivotConstants.kPivotFromBase - ShooterPivotConstants.kShooterRadius * ShooterPivotConstants.kShooterRadius) / (2 * ShooterPivotConstants.kPivotFromBase * m_elevator.getLength())) * 180 / Math.PI));
    for (int i = 0; i < ShooterPivotConstants.kShooterSize; i++) {
      m_shooter[i].setAngle(ShooterPivotConstants.kStartShooterDegrees + (5 * i) + Units.radiansToDegrees(Math.acos(((m_elevator.getLength() * m_elevator.getLength()) - ((ShooterPivotConstants.kPivotFromBase * ShooterPivotConstants.kPivotFromBase) + (ShooterPivotConstants.kShooterRadius * ShooterPivotConstants.kShooterRadius))) / (- 2 * ShooterPivotConstants.kPivotFromBase * ShooterPivotConstants.kShooterRadius))));
      m_shooterVoid[i].setAngle(ShooterPivotConstants.kStartShooterDegrees + (5 * i) + Units.radiansToDegrees(Math.acos(((m_elevator.getLength() * m_elevator.getLength()) - ((ShooterPivotConstants.kPivotFromBase * ShooterPivotConstants.kPivotFromBase) + (ShooterPivotConstants.kShooterRadius * ShooterPivotConstants.kShooterRadius))) / (- 2 * ShooterPivotConstants.kPivotFromBase * ShooterPivotConstants.kShooterRadius))));
    }
  }
}
