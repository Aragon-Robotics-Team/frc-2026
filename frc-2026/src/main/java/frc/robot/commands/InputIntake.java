// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystem.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InputIntake extends Command {
  /** Creates a new InputIntake. */
  private Intake m_intake;
  private double m_speed;

  private InputIntakeSendable m_inputIntakeSendable = new InputIntakeSendable();

  public InputIntake(Intake intake, double speed) {
    m_intake = intake;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(m_speed * IntakeConstants.kSpeedMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Sendable getSendable() {
    return m_inputIntakeSendable;
  }

  private class InputIntakeSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Input Intake");
      builder.addDoubleProperty("speed", ()-> m_speed, null/*(double speed)->m_speed=speed*/);
    }
  }
}
