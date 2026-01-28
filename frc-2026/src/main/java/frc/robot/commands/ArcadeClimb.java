// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeClimb extends Command {

  private Climb m_climb;
  private Joystick m_joystick;
  private double m_speed;

  private ArcadeClimbSendable m_arcadeClimbSendable = new ArcadeClimbSendable();

  /** Creates a new ArcadeClimb. */
  public ArcadeClimb(Climb climb, Joystick joystick) {

    m_climb = climb;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_joystick.getRawAxis(IOConstants.kClimbJoystickAxis);
    m_speed *= ClimbConstants.kClimbSpeedMultiplier;
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

  public ArcadeClimbSendable getArcadeClimbSendable() {
    return m_arcadeClimbSendable;
  }

  private class ArcadeClimbSendable implements Sendable {
    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Arcade Climb");
      builder.addDoubleProperty("Current Tick Position", () -> m_climb.getTickPosition(), null);
    }
  }
}
