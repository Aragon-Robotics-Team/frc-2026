// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.subsystems.IntakePivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDIntakePivot extends Command {
  /** Creates a new PIDIntakePivot. */
  private IntakePivot m_intakePivot; // subsystem that is used
  
  private PIDController m_PIDController = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD); // built in PID maker thing
  private TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakePivotConstants.kMaxVelocity, IntakePivotConstants.kMaxAcceleration));

  private TrapezoidProfile.State m_startPoint;
  private TrapezoidProfile.State m_endPoint;
  private TrapezoidProfile.State m_betweenPoint;

  private double m_targetTicks; // desired goalpoint
  private double m_speed; // the input speed variable
  private Timer m_timer = new Timer();

  private PIDIntakePivotSendable m_PIDIntakePivotSendable = new PIDIntakePivotSendable();

  public PIDIntakePivot(IntakePivot intakePivot, double targetTicks) {
    m_intakePivot = intakePivot;
    m_targetTicks = targetTicks; // passing in values

    m_timer.restart();
    m_startPoint = new TrapezoidProfile.State(m_intakePivot.getEncoderTicks(), m_intakePivot.getSpeed());
    m_endPoint = new TrapezoidProfile.State(m_targetTicks, 0);

    m_PIDController.setIZone(IntakePivotConstants.kPIDIZone); // intergral application zones
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_betweenPoint = m_trapezoidProfile.calculate(m_timer.get() , m_startPoint, m_endPoint);
    m_speed = m_PIDController.calculate(m_intakePivot.getEncoderTicks(), m_betweenPoint.position); // calculating the speed
    m_intakePivot.setSpeed(m_speed); //setting the speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakePivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

public PIDIntakePivotSendable getPIDIntakePivotSendable() {
    return m_PIDIntakePivotSendable;
  }

  private class PIDIntakePivotSendable implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("PIDIntakePivot");
      builder.addDoubleProperty("Speed", () -> m_intakePivot.getSpeed(), null);
      builder.addDoubleProperty("Target Ticks", () -> m_targetTicks, (double ticks) -> m_targetTicks = ticks);
      builder.addDoubleProperty("Current Ticks", () -> m_intakePivot.getEncoderTicks(), null);
    }
  }
}
