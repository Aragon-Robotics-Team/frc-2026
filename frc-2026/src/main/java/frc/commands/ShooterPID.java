// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterPID extends Command {
  private Shooter m_shooter;
  private PIDController m_pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  private double m_accelerationRPM;

  private ShooterPIDSendable m_shooterPIDSendable = new ShooterPIDSendable();

  /** Creates a new ShooterPID. */
  public ShooterPID(Shooter shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_accelerationRPM = m_pidController.calculate(m_shooter.getLeftRPM(), ShooterConstants.kTargetRPM);
    //double m_bottom_acceleration_rpm = m_pidController.calculate(9000, targetRPM);
    // System.out.println(m_shooter.getBottomRPM()+m_bottom_acceleration_rpm);
    // System.out.println("rpm current" + m_shooter.getBottomRPM());
    // System.out.println("rpm acc" + m_bottom_acceleration_rpm);
    System.out.println(Shooter.WheelRPMtoDutyCycle(m_accelerationRPM));
    System.out.println("duty cycle" + Shooter.WheelRPMtoDutyCycle(m_shooter.getLeftRPM()+ m_accelerationRPM));
    
    m_shooter.setRPM(ShooterConstants.kTargetRPM + m_accelerationRPM);
    //m_shooter.setBottomDutyCycle(0.7);
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
      builder.addDoubleProperty("Target RPM", () -> ShooterConstants.kTargetRPM, null);
      builder.addDoubleProperty("Left Shooter RPM", () -> m_shooter.getLeftRPM(), null);
      builder.addDoubleProperty("Right Shooter RPM", () -> m_shooter.getRightRPM(), null);
    }
  }
}
