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
import frc.robot.commands.ClimbPID;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Climb;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);
  private JoystickButton m_climbPIDButton = new JoystickButton(m_driverJoystick, IOConstants.kClimbPIDButtonID);
  private JoystickButton m_arcadeClimbButton = new JoystickButton(m_driverJoystick, IOConstants.kArcadeClimbButtonID);
  private Climb m_climb = new Climb();
  
  private ArcadeClimb m_arcadeClimb = new ArcadeClimb(m_climb, m_driverJoystick);
  private ClimbPID m_climbPID = new ClimbPID(m_climb, ClimbConstants.kTickGoal);

  public RobotContainer() {
    SendableRegistry.add(m_arcadeClimb.getArcadeClimbSendable(), "Arcade Climb");
    Shuffleboard.getTab("SmartDashboard").add(m_arcadeClimb.getArcadeClimbSendable()).withWidget("Arcade Climb");
    SendableRegistry.add(m_climbPID.getClimbPIDSendable(), "Climb PID");
    Shuffleboard.getTab("SmartDashboard").add(m_climbPID.getClimbPIDSendable()).withWidget("Climb PID");
    configureBindings();
  }

  private void configureBindings() {
    m_arcadeClimbButton.whileTrue(m_arcadeClimb);
    m_climbPIDButton.whileTrue(m_climbPID);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
