// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterPivotToPosition extends Command {
  /** Creates a new ShooterPivotToPosition. */
  private ShooterPivot m_shooterPivot;
  private double m_targetHeight;

  private ShooterPivotToPositionSendable m_shooterPivotToPositionSendable = new ShooterPivotToPositionSendable();

  public ShooterPivotToPosition(ShooterPivot shooterPivot, double targetHeight) {
    m_shooterPivot = shooterPivot;
    m_targetHeight = targetHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterPivot.setPosition(m_targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterPivot.setPosition(m_shooterPivot.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public ShooterPivotToPositionSendable getArmArcadeSendable() {
    return m_shooterPivotToPositionSendable;
  }

  private class ShooterPivotToPositionSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ShooterPivot to Position");
      builder.addDoubleProperty("Current Position", () -> m_shooterPivot.getPosition(), null);
      builder.addDoubleProperty("Target Position", () -> m_targetHeight, (double setHeight) -> m_targetHeight = setHeight);
    }
  }
}