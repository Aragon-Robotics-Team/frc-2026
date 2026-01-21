// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeIntakePivot extends Command {
  /** Creates a new ArcadeIntakePivot. */
  private IntakePivot m_intakePivot; // the intake subsystem
  private Joystick m_joystick; // the joystick input
  private double m_speed; // speed for command

  private IntakePivotArcadeSendable m_IntakePivotArcadeSendable = new IntakePivotArcadeSendable();
  
  public ArcadeIntakePivot(IntakePivot intakePivot, Joystick joystick) {
    m_intakePivot = intakePivot;
    m_joystick = joystick; // passing in values
    addRequirements(m_intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_joystick.getRawAxis(IOConstants.kIntakePivotJoystickAxis); // getting input from joystick
    m_speed *= IntakePivotConstants.kArcadeSpeedMultiplier; // adjusting speed
    m_intakePivot.setSpeed(m_speed); // setting the speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakePivot.setSpeed(0); // in case of bad stuff and it stops
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public IntakePivotArcadeSendable getIntakePivotArcadeSendable() {
    return m_IntakePivotArcadeSendable;
  } // sending the sendable

  private class IntakePivotArcadeSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("IntakePivotArcade");
      builder.addDoubleProperty("Speed", () -> m_intakePivot.getSpeed(), (double speed) -> m_intakePivot.setSpeed(speed));
      builder.addDoubleProperty("Ticks", () -> m_intakePivot.getEncoderTicks(), null);
    }
  } // Making the dashboard stuff
}
