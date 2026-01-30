// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeIntakePivot;
import frc.robot.commands.PIDIntakePivot;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.commands.ArcadeShooterPivot;
import frc.robot.commands.ShooterPivotToPosition;
import frc.robot.constants.ShooterPivotConstants;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  private Mechanism2d m_mech = new Mechanism2d(20, 20);

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);

  private JoystickButton m_intakePivotButton = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonIntakePivot);
  private IntakePivot m_intakePivot = new IntakePivot(m_mech);
  private ArcadeIntakePivot m_arcadeIntakePivot = new ArcadeIntakePivot(m_intakePivot, m_secondaryJoystick);
  private PIDIntakePivot m_PIDIntakePivot = new PIDIntakePivot(m_intakePivot, IntakePivotConstants.kTargetTicks);

  private JoystickButton m_buttonShooterPivot = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonShooterPivotID);
  private ShooterPivot m_shooterPivot = new ShooterPivot(m_mech);
  private ArcadeShooterPivot m_arcadeShooterPivot = new ArcadeShooterPivot(m_shooterPivot, m_secondaryJoystick);
  private ShooterPivotToPosition m_shooterPivotToPosition = new ShooterPivotToPosition(m_shooterPivot, ShooterPivotConstants.kToPositionTargetHeight);

  private JoystickButton m_buttonLaunchGampiece = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonLaunchGamepiece);

  public RobotContainer() {
    
    m_intakePivot.setDefaultCommand(m_arcadeIntakePivot);
    m_shooterPivot.setDefaultCommand(m_arcadeShooterPivot);

    SmartDashboard.putData("Mechanism 2D Field Simulation", m_mech);

    SendableRegistry.add(m_arcadeIntakePivot.getIntakePivotArcadeSendable(), "IntakePivotArcade");
    Shuffleboard.getTab("SmartDashboard").add(m_arcadeIntakePivot.getIntakePivotArcadeSendable()).withWidget("IntakePivotArcade");
    SendableRegistry.add(m_PIDIntakePivot.getPIDIntakePivotSendable(), "IntakePivotPID");
    Shuffleboard.getTab("SmartDashboard").add(m_PIDIntakePivot.getPIDIntakePivotSendable()).withWidget("IntakePivotPID");
    configureBindings();
  }

  private void configureBindings() {
    m_intakePivotButton.whileTrue(m_PIDIntakePivot);
    m_buttonShooterPivot.whileTrue(m_shooterPivotToPosition);
    m_buttonLaunchGampiece.onTrue(m_shooterPivot.launch());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
