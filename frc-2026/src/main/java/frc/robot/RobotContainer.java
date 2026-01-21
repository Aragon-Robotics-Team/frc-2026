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
import frc.commands.ShooterPID;
import frc.robot.constants.IOConstants;
import frc.subsystems.Shooter;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);

  private Shooter m_shooter = new Shooter();
  private ShooterPID m_shooterPID = new ShooterPID(m_shooter);

  private JoystickButton m_buttonShooter = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonNumShooter);

  public RobotContainer() {
    m_shooter.setDefaultCommand(m_shooter.idle());
    SendableRegistry.add(m_shooterPID.getSendable(), "ShooterPID");
    Shuffleboard.getTab("SmartDashboard").add(m_shooterPID.getSendable()).withWidget("ShooterPID");
    configureBindings();
  }

  private void configureBindings() {
    m_buttonShooter.whileTrue(m_shooterPID);
    m_buttonShooter.onFalse(m_shooter.idle());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
