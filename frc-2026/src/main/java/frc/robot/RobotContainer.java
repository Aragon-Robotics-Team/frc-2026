// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShooterPID;
//import frc.robot.commands.SmartDashboardShooter;
import frc.robot.constants.IOConstants;
import frc.robot.subsystem.Shooter;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);
  private JoystickButton m_joystickButton1 = new JoystickButton(m_driverJoystick, IOConstants.kJoystickButton1ID);
  private JoystickButton m_joystickButton2 = new JoystickButton(m_driverJoystick, IOConstants.kJoystickButton2ID);

  private Shooter m_shooter = new Shooter();
  private ShooterPID m_shooterPID = new ShooterPID(m_shooter);
  //private SmartDashboardShooter m_dashboardShooter = new SmartDashboardShooter(m_shooter);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_joystickButton1.whileTrue(m_shooterPID);
    //m_joystickButton2.whileTrue(m_dashboardShooter);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
