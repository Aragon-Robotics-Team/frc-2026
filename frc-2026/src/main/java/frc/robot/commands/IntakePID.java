// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystem.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePID extends Command {

  private Intake m_intake;
  private PIDController m_pidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

  private TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_goalState;
  private TrapezoidProfile.State m_setpointState;
  private Timer m_timer = new Timer();

  private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);
  private double m_targetRPM;
  private double m_acclerationRPM;
  private double m_speed;

  private IntakePIDSendable m_intakePIDSendable = new IntakePIDSendable();

  /** Creates a new IntakePID. */
  public IntakePID(Intake intake, double targetRPM) {

    m_intake = intake;
    m_targetRPM = targetRPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_startState = new TrapezoidProfile.State(0, m_intake.getIntakeRPM());
    m_goalState = new TrapezoidProfile.State(Integer.MAX_VALUE, m_targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_setpointState = m_trapezoidProfile.calculate(m_timer.get(), m_startState, m_goalState);
    m_acclerationRPM = m_pidController.calculate(m_intake.getIntakeRPM(), m_setpointState.velocity);

    m_speed = m_feedForward.calculateWithVelocities(m_intake.getIntakeRPM(), m_setpointState.velocity);
    m_intake.setIntakeRPM(m_intake.getIntakeRPM() + m_acclerationRPM + m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private class IntakePIDSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Intake PID");
      builder.addDoubleProperty("RPM of Intake", () -> m_intake.getIntakeRPM(), null);
      builder.addDoubleProperty("Target Intake PID RPM ", () -> m_targetRPM, (double targetRPM) -> m_targetRPM = targetRPM);
    }
  }

  public IntakePIDSendable getIntakePIDSendable() {
    return m_intakePIDSendable;
  }
}
