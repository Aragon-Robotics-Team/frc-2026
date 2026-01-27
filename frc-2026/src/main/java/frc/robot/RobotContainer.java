// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeIntakePivot;
import frc.robot.commands.PIDIntakePivot;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.IntakePivot;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);

  private JoystickButton m_intakePivotButton = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonIntakePivot);

  private IntakePivot m_intakePivot = new IntakePivot();
  private ArcadeIntakePivot m_arcadeIntakePivot = new ArcadeIntakePivot(m_intakePivot, m_secondaryJoystick);
  private PIDIntakePivot m_PIDIntakePivot = new PIDIntakePivot(m_intakePivot, IntakePivotConstants.kTargetTicks);

  public RobotContainer() {
    
    m_intakePivot.setDefaultCommand(m_PIDIntakePivot);

    SendableRegistry.add(m_arcadeIntakePivot.getIntakePivotArcadeSendable(), "IntakePivotArcade");
    Shuffleboard.getTab("SmartDashboard").add(m_arcadeIntakePivot.getIntakePivotArcadeSendable()).withWidget("IntakePivotArcade");
    SendableRegistry.add(m_PIDIntakePivot.getPIDIntakePivotSendable(), "IntakePivotPID");
    Shuffleboard.getTab("SmartDashboard").add(m_PIDIntakePivot.getPIDIntakePivotSendable()).withWidget("IntakePivotPID");
    configureBindings();
  }

  private void configureBindings() {
    m_intakePivotButton.whileTrue(m_PIDIntakePivot);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
