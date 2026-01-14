// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystick extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController m_joystick;
  /** Creates a new SwerveJoystick. */
  public SwerveJoystick(SwerveDrive swerveDrive, XboxController joystick) {
    m_swerveDrive = swerveDrive;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeed = new ChassisSpeeds(-m_joystick.getLeftY() * DriveConstants.kMaxMotorSpeed, -m_joystick.getLeftX() * DriveConstants.kMaxMotorSpeed, -m_joystick.getRightX() * DriveConstants.kMaxMotorSpeed);
    m_swerveDrive.driveRobot(chassisSpeed);
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
