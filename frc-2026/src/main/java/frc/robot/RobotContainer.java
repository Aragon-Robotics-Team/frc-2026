// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeShooterPivot;
import frc.robot.commands.ShooterPivotToPosition;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ShooterPivotConstants;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  private Joystick m_driverJoystick = new Joystick(IOConstants.kDriverJoystickID);
  private Joystick m_secondaryJoystick = new Joystick(IOConstants.kSecondaryJoystickID);

  private JoystickButton m_buttonShooterPivot = new JoystickButton(m_secondaryJoystick, IOConstants.kButtonShooterPivotID);

  private ShooterPivot m_shooterPivot = new ShooterPivot();

  private ArcadeShooterPivot m_arcadeShooterPivot = new ArcadeShooterPivot(m_shooterPivot, m_secondaryJoystick);
  private ShooterPivotToPosition m_shooterPivotToPosition = new ShooterPivotToPosition(m_shooterPivot, ShooterPivotConstants.kToPositionTargetHeight);

  public RobotContainer() {
    m_shooterPivot.setDefaultCommand(m_arcadeShooterPivot);
    configureBindings();
  }

  private void configureBindings() {
    m_buttonShooterPivot.whileTrue(m_shooterPivotToPosition);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
