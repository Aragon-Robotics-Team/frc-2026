// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystick extends Command {

  private final SwerveDrive m_swerveDrive;
  private XboxController m_joystick;
  private boolean m_fieldRelative = true;
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

  public void toggleRobotRelativeFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
  }

  public InstantCommand toggleRobotRelativeFieldRelativeCommand(){
    return new InstantCommand(this::toggleRobotRelativeFieldRelative);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vx = -m_joystick.getLeftY() * DriveConstants.kMaxTranslationalMetersPerSecond * DriveConstants.kSpeedLimiter;
    double vy = -m_joystick.getLeftX() * DriveConstants.kMaxTranslationalMetersPerSecond * DriveConstants.kSpeedLimiter;
    double omega = -m_joystick.getRightX() * DriveConstants.kMaxTranslationalMetersPerSecond;
    ChassisSpeeds m_chassisSpeed = new ChassisSpeeds(vx, vy, omega);
    
    if(m_fieldRelative){
      m_chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeed, m_swerveDrive.getMeasuredAngle());
      SmartDashboard.putBoolean("field relative", true);
      SmartDashboard.putBoolean("robot relative", false);
    }
    else{
      SmartDashboard.putBoolean("robot relative", true);
      SmartDashboard.putBoolean("field relative", false);
    }
    m_swerveDrive.driveRobot(m_chassisSpeed);
    Logger.recordOutput("chassis speed", m_chassisSpeed);
    Logger.recordOutput("vx", vx);
    Logger.recordOutput("vy", -m_joystick.getRawAxis(DriveConstants.kLeftXAxisID) * DriveConstants.kMaxTranslationalMetersPerSecond);
    Logger.recordOutput("rotation", -m_joystick.getRawAxis(DriveConstants.kRightXAxisID) * DriveConstants.kMaxTranslationalMetersPerSecond);
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
