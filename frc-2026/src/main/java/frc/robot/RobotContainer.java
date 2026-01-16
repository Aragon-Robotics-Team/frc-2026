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
import frc.robot.commands.ArcadeClimb;
import frc.robot.commands.ClimbToPosition;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Climb;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);

  private Climb m_climb = new Climb();

  private ArcadeClimb m_arcadeClimb = new ArcadeClimb(m_climb, m_secondaryJoystick);

  private ClimbToPosition m_climbToStowed = new ClimbToPosition(m_climb, ClimbConstants.kTicksStowed);
  private ClimbToPosition m_climbToExtended = new ClimbToPosition(m_climb, ClimbConstants.kTicksExtended);

  private JoystickButton m_buttonClimbToStowed = new JoystickButton(m_secondaryJoystick, IOConstants.kClimbToStowedButtonID);
  private JoystickButton m_buttonClimbToExtended = new JoystickButton(m_secondaryJoystick, IOConstants.kClimbToExtendedButtonID);

  public RobotContainer() {
    m_climb.setDefaultCommand(m_arcadeClimb);

    SendableRegistry.add(m_arcadeClimb.getSendable(), "ArcadeClimb");
    Shuffleboard.getTab("SmartDashboard").add(m_arcadeClimb.getSendable()).withWidget("ArcadeClimb");

    
    SendableRegistry.add(m_climbToStowed.getSendable(), "ClimbToStowed");
    Shuffleboard.getTab("SmartDashboard").add(m_climbToStowed.getSendable()).withWidget("ClimbToStowed");
    
    SendableRegistry.add(m_climbToExtended.getSendable(), "ClimbToExtended");
    Shuffleboard.getTab("SmartDashboard").add(m_climbToExtended.getSendable()).withWidget("ClimbToExtended");
    
    configureBindings();
  }

  private void configureBindings() {
    m_buttonClimbToStowed.whileTrue(m_climbToStowed);
    m_buttonClimbToExtended.whileTrue(m_climbToExtended);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
