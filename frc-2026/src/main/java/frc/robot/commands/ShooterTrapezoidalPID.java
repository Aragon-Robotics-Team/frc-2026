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
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterTrapezoidalPID extends Command {
  private Shooter m_shooter;

  private TrapezoidProfile m_trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ShooterConstants.kMaxVel, ShooterConstants.kMaxAccel));
  private Timer m_timer = new Timer();
  private PIDController m_pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  
  private double m_accelerationRPM;
  private double m_targetRPM;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_goalState;
  private TrapezoidProfile.State m_setpoint;

  private ShooterPIDSendable m_shooterPIDSendable = new ShooterPIDSendable();

  /** Creates a new ShooterPID. */
  public ShooterTrapezoidalPID(Shooter shooter, double targetRPM) {
    m_shooter = shooter;
    m_targetRPM = targetRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_startState = new TrapezoidProfile.State(0, m_shooter.getLeftShooterRPM());
    m_goalState = new TrapezoidProfile.State(Integer.MAX_VALUE, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_setpoint = m_trapezoidProfile.calculate(m_timer.get(), m_startState, m_goalState);
    m_accelerationRPM = m_pidController.calculate(m_shooter.getLeftShooterRPM(), m_setpoint.velocity);
    //System.out.println(Shooter.WheelRPMtoDutyCycle(m_accelerationRPM));
    //System.out.println("duty cycle" + Shooter.WheelRPMtoDutyCycle(m_shooter.getLeftShooterRPM()+ m_accelerationRPM));

    // m_shooter.setRPM(m_targetRPM + m_accelerationRPM);
    // System.out.println("State setpoint: " + m_speedState.position);
    // System.out.println("Speed RPM: " + m_shooter.getLeftShooterRPM());
    // System.out.println("PIDoutput: " + m_accelerationRPM);
    // System.out.println("Set speed: " + m_accelerationRPM + m_shooter.getLeftShooterRPM());
    // System.out.println();
    // m_shooter.setRPM(m_speedState.position);
    m_shooter.setRPM(m_accelerationRPM + m_shooter.getLeftShooterRPM());   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setDutyCycle(0);
    m_shooter.setDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public ShooterPIDSendable getSendable() {
    return m_shooterPIDSendable;
  }

  private class ShooterPIDSendable implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ShooterPID");
      builder.addDoubleProperty("Target RPM", () -> m_targetRPM, (double targetRPM) -> m_targetRPM = targetRPM);
      builder.addDoubleProperty("Left Shooter RPM", () -> m_shooter.getLeftShooterRPM(), null);
      builder.addDoubleProperty("Right Shooter RPM", () -> m_shooter.getRightShooterRPM(), null);
    }
  }
}
