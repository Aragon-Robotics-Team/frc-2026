// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IOConstants;
import frc.robot.constants.ShooterPivotConstants;
import frc.robot.subsystems.ShooterPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeShooterPivot extends Command {
  /** Creates a new ArcadeShooterPivot. */
  private ShooterPivot m_shooterPivot;
  private Joystick m_joystick;
  private double m_position;

  private ArcadeShooterPivotSendable m_arcadeShooterPivotSendable = new ArcadeShooterPivotSendable();

  public ArcadeShooterPivot(ShooterPivot shooterPivot, Joystick joystick) {
    m_shooterPivot = shooterPivot;
    m_joystick = joystick;
    m_position = shooterPivot.getPosition();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_joystick.getRawAxis(IOConstants.kShooterPivotAxis)!=0) {
      m_position += ShooterPivotConstants.kArcadeSpeedMultiplier * m_joystick.getRawAxis(IOConstants.kShooterPivotAxis);
      m_shooterPivot.setPosition(m_position);
    } else {
      m_position = m_shooterPivot.getPosition();
      m_shooterPivot.setPosition(m_shooterPivot.getPosition());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_position = m_shooterPivot.getPosition();
    m_shooterPivot.setPosition(m_shooterPivot.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public Sendable getArcadeShooterPivotSendable() {
    return m_arcadeShooterPivotSendable;
  }

  private class ArcadeShooterPivotSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Arcade Shooter Pivot");
      builder.addDoubleProperty("Current Position", () -> m_shooterPivot.getPosition(), null);
      builder.addDoubleProperty("Going To Position", () -> m_position, null);
    }
  }
}
