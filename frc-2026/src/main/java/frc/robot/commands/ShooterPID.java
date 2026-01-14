// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystem.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterPID extends Command {

  private Shooter m_shooter = new Shooter();
  private PIDController m_pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  private double m_bottomTicksGoal;
  private double m_UpperTicksGoal;
  private double m_currentBottomTickPosition;
  private double m_currentTopTickPosition;
  private double m_bottomMotorSpeed;
  private double m_UpperMotorSpeed;
  private double m_bottomMotorRotation;
  private double m_UpperMotorRotation;
  private double m_bottomWheelRotation;
  private double m_UpperWheelRotation;
  

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
    m_bottomWheelRotation = SmartDashboard.getNumber("Bottom Shooter PID set speed", 0);
    m_UpperWheelRotation = SmartDashboard.getNumber("Upper Shooter PID set speed", 0);

    m_bottomMotorRotation = m_bottomWheelRotation * ShooterConstants.kBottomWheelToMotorConvertion;
    m_UpperMotorRotation = m_UpperWheelRotation * ShooterConstants.kUpperWheelToMotorConvertion;

    m_bottomTicksGoal = m_bottomMotorRotation * ShooterConstants.kTicksPerRotation;
    m_UpperTicksGoal = m_UpperMotorRotation * ShooterConstants.kTicksPerRotation;

    m_currentBottomTickPosition = m_shooter.getBottomEncoderVelocity();
    m_currentTopTickPosition = m_shooter.getTopEncoderVelocity();

    m_bottomMotorSpeed = m_pidController.calculate(m_currentBottomTickPosition, m_bottomTicksGoal);
    m_UpperMotorSpeed = m_pidController.calculate(m_currentTopTickPosition, m_UpperTicksGoal);

    m_shooter.setBottomSpeed(m_bottomMotorSpeed);
    m_shooter.setUpperSpeed(m_UpperMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setBottomSpeed(0);
    m_shooter.setUpperSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
