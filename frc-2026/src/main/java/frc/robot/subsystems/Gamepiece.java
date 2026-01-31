// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GamepieceConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.ShooterConstants;

public class Gamepiece extends SubsystemBase {
  /** Creates a new Gamepiece. */
  private ShooterPivot m_shooterPivot;
  private Shooter m_shooter;

  private static int id = 0;
  private Mechanism2d m_mech;
  private MechanismRoot2d m_gamepiece;
  private double m_angleRads;
  
  private double m_xOffset;
  private double m_yOffset;

  private double m_initialV;
  private double m_initialXV;
  private double m_initialYV;

  private double m_xAccel;
  private double m_yAccel;

  private double m_time;
  private MechanismLigament2d[] m_gamepieceRender;

  public Gamepiece(Mechanism2d mech, ShooterPivot shooterPivot, Shooter shooter) {
    m_shooterPivot = shooterPivot;
    m_shooter = shooter;
    m_mech = mech;

    m_xOffset = m_shooterPivot.getSimX();
    m_yOffset = m_shooterPivot.getSimY();

    m_gamepiece = m_mech.getRoot(id + "fuel", m_xOffset, m_yOffset);

    m_gamepieceRender = new MechanismLigament2d[9];
    for (int i = 0; i < 9; i++) {
      m_gamepieceRender[i] = m_gamepiece.append(new MechanismLigament2d(id + "" + i + "", GamepieceConstants.kGamepieceRadius, i * 40.0));
    }

    m_angleRads = m_shooterPivot.getLaunchAngle();
    m_initialV = m_shooter.getLeftShooterDutyCycle() * 10000.0 * 2.0 * Math.PI * ShooterConstants.kRollerRadius;
    m_initialXV = Math.cos(m_angleRads) * m_initialV;
    m_initialYV = Math.sin(m_angleRads) * m_initialV;

    m_xAccel = 0;
    m_yAccel = GamepieceConstants.kGravity;

    m_time = 0;
    id++;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    m_gamepiece.setPosition(m_xOffset + m_initialXV * m_time + (1.0 / 2.0) * m_xAccel * m_time * m_time, m_yOffset + m_initialYV * m_time + (1.0 / 2.0) * m_yAccel * m_time * m_time);
    m_time += IntakePivotConstants.kLoopTime;
  }
}
