// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ShooterTestJig;

public class RobotContainer {

  private ShooterTestJig m_shooter = new ShooterTestJig();
  private Shoot m_shoot = new Shoot(m_shooter);

  public RobotContainer() {
    m_shooter.setDefaultCommand(m_shoot);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
