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
import frc.robot.commands.ArcadeIntake;
import frc.robot.commands.IntakePID;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystem.Intake;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);
  private JoystickButton m_arcadeIntakeButton = new JoystickButton(m_driverJoystick, IOConstants.karcadeIntakeButtonID);
  private JoystickButton m_intakePIDButton = new JoystickButton(m_driverJoystick, IOConstants.kintakePIDButtonID);
  
  private Intake m_intake = new Intake(); 

  private ArcadeIntake m_arcadeIntake = new ArcadeIntake(m_intake);
  private IntakePID m_intakePID = new IntakePID(m_intake, IntakeConstants.kTargetRPM);

  public RobotContainer() {
    SendableRegistry.add(m_arcadeIntake.getArcadeIntakeSendable(), "Arcade Intake");
    Shuffleboard.getTab("SmartDashboard").add(m_arcadeIntake.getArcadeIntakeSendable()).withWidget("Arcade Intake");
    SendableRegistry.add(m_intakePID.getIntakePIDSendable(), "Intake PID");
    Shuffleboard.getTab("SmartDashboard").add(m_intakePID.getIntakePIDSendable()).withWidget("Intake PID");
    configureBindings();
  }

  private void configureBindings() {
    m_arcadeIntakeButton.whileTrue(m_arcadeIntake);
    m_intakePIDButton.whileTrue(m_intakePID);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
