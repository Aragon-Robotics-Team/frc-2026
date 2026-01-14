// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.InputIntake;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystem.Intake;

public class RobotContainer {

  private Intake m_intake = new Intake();
  
  private InputIntake m_insertIntake = new InputIntake(m_intake, IntakeConstants.kInSpeed);
  private InputIntake m_outputIntake = new InputIntake(m_intake, IntakeConstants.kOutSpeed);

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);
  private JoystickButton m_joystickButton1 = new JoystickButton(m_driverJoystick, IOConstants.kDriverJoystickButton1);
  private JoystickButton m_joystickButton2 = new JoystickButton(m_driverJoystick, IOConstants.kDriverJoystickButton2);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_joystickButton1.whileTrue(m_insertIntake);
    m_joystickButton2.whileTrue(m_outputIntake);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
