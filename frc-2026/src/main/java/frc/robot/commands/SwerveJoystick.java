// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystick extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController m_joystick;

  private final Vision m_vision;

  /** Creates a new SwerveJoystick. */
  public SwerveJoystick(SwerveDrive swerveDrive, Vision vision, XboxController joystick) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds m_chassisSpeed = new ChassisSpeeds(m_joystick.getLeftY() * DriveConstants.kMaxMotorSpeed, m_joystick.getLeftX() * DriveConstants.kMaxMotorSpeed, m_joystick.getRightX() * DriveConstants.kMaxMotorSpeed);
    //ChassisSpeeds m_chassisSpeed = new ChassisSpeeds(0.2, 0, 0);
    m_swerveDrive.driveRobot(ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeed, m_swerveDrive.getMeasuredAngle()));
    Logger.recordOutput("chassis speed", m_chassisSpeed);
    Logger.recordOutput("vx", -m_joystick.getRawAxis(DriveConstants.kLeftYAxisID) * DriveConstants.kMaxMotorSpeed);
    Logger.recordOutput("vy", -m_joystick.getRawAxis(DriveConstants.kLeftXAxisID) * DriveConstants.kMaxMotorSpeed);
    Logger.recordOutput("rotation", -m_joystick.getRawAxis(DriveConstants.kRightXAxisID) * DriveConstants.kMaxMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
