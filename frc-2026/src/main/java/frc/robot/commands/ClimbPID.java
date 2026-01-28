// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPID extends Command {

  private PIDController m_pidController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);
  private Climb m_climb;

  private double m_radianSetpoint;
  private double m_PIDspeed;
  private double m_feedForwardSpeed;

  private TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ClimbConstants.kMaxVelocity, ClimbConstants.kMaxAcceleration));
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_goalState;
  private TrapezoidProfile.State m_setpointState;
  private Timer m_timer = new Timer();

  private ArmFeedforward m_feedForward = new ArmFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV, ClimbConstants.kA);

  private ClimbPIDSendable m_climbPIDSendable = new ClimbPIDSendable();
  
  /** Creates a new ClimbPID. */
  public ClimbPID(Climb climb, double tickSetpoint) {

    m_climb = climb;
    m_radianSetpoint = (tickSetpoint/ClimbConstants.kTicksPerRotation) * ClimbConstants.kRadiansPerRotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_startState = new TrapezoidProfile.State(m_climb.getRadianPosition(), 0);
    m_goalState = new TrapezoidProfile.State(m_radianSetpoint, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_setpointState = m_trapezoidProfile.calculate(m_timer.get(), m_startState, m_goalState);
    m_PIDspeed = m_pidController.calculate(m_climb.getTickPosition(), m_setpointState.position);
    m_feedForwardSpeed = m_feedForward.calculate(m_setpointState.position, m_setpointState.velocity);
    m_climb.setSpeed(m_PIDspeed + m_feedForwardSpeed);
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

  public ClimbPIDSendable getClimbPIDSendable() {
    return m_climbPIDSendable;
  }

  private class ClimbPIDSendable implements Sendable {
    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Climb PID");
      builder.addDoubleProperty("Current Tick Position", () -> m_climb.getTickPosition(), null);
      builder.addDoubleProperty("Current Radian Position", () -> m_climb.getRadianPosition(), null);

    }
  }
}
