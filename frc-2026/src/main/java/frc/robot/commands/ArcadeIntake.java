// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IOConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystem.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeIntake extends Command {

  private Intake m_intake;
  private double m_rpm;

  private ArcadeIntakeSendable m_arcadeIntakeSendable = new ArcadeIntakeSendable();

  /** Creates a new ArcadeIntake. */
  public ArcadeIntake(Intake intake) {

    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Set Arcade Intake RPM", IntakeConstants.kTargetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rpm = SmartDashboard.getNumber("Set Arcade Intake RPM", IOConstants.kSmartDashBoardDefaultValue);
    m_intake.setDutyCycle(m_rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private class ArcadeIntakeSendable implements Sendable{
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Arcade Intake");
      builder.addDoubleProperty("RPM of Intake", () -> m_intake.getRPM(), null);
    }

  }

  public ArcadeIntakeSendable getArcadeIntakeSendable() {
    return m_arcadeIntakeSendable;
  }
}
