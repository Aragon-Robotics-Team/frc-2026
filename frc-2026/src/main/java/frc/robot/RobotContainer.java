// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {

  private XboxController m_driverJoystick = new XboxController(IOConstants.kDriverJoystickID);
  private XboxController m_secondaryJoystick = new XboxController(IOConstants.kSecondaryJoystickID);

  private SwerveDrive m_swerveDrive = new SwerveDrive();
  private SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerveDrive, m_driverJoystick);


  private final JoystickButton m_resetHeadingButton = new JoystickButton(m_driverJoystick, IOConstants.kResetHeadingButtonID);
  private final InstantCommand m_resetHeadingCommand = m_swerveDrive.resetHeadingCommand();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_swerveDrive.setDefaultCommand(m_swerveJoystick);
     m_resetHeadingButton.whileTrue(m_resetHeadingCommand); // button 4, y button
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
