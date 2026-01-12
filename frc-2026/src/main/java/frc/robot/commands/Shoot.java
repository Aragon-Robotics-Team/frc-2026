// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterTestJigConstants;
import frc.robot.subsystems.ShooterTestJig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {

  private ShooterTestJig m_shooter;
  private double m_speed = 0;

  /** Creates a new Shoot. */
  public Shoot(ShooterTestJig shooter) {
    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Relative Speed", 0);
    m_speed = SmartDashboard.getNumber("Shooter Relative Speed", 0);
    m_shooter.setMainSpeed(m_speed*ShooterTestJigConstants.kMainGearRatio);
    m_shooter.setUpperSpeed(m_speed*ShooterTestJigConstants.kUpperGearRatio);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMainSpeed(0);
    m_shooter.setUpperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
