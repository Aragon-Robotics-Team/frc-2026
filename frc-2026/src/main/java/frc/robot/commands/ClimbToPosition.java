// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbToPosition extends Command {
  /** Creates a new ClimbToPosition. */
  
  private Climb m_climb;
  private PIDController m_PIDController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);
  private double m_setpoint;
  private double m_speed;
  private ClimbToPositionSendable m_climbToPositionSendable = new ClimbToPositionSendable();

  public ClimbToPosition(Climb climb, double setpoint) {
    m_climb = climb;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_PIDController.calculate(m_climb.getTicks(), m_setpoint);
    m_climb.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public ClimbToPositionSendable getSendable() {
    return m_climbToPositionSendable;
  }

  private class ClimbToPositionSendable implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ArmPID");
      builder.addDoubleProperty("Speed", () -> m_climb.getSpeed(), null);
      builder.addDoubleProperty("Ticks", () -> m_climb.getTicks(), null);
      builder.addDoubleProperty("Target In Ticks", () -> m_setpoint, null);
    }
  }
}
