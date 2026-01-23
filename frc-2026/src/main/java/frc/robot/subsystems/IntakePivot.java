// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private TalonFX m_motor = new TalonFX(IntakePivotConstants.kMotorID); // creating the objects on the subsystem
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderChannel);

  private Mechanism2d m_mech = new Mechanism2d(5 , 5);
  private MechanismRoot2d m_pivot = m_mech.getRoot("Pivot", 1, 1);
  private MechanismLigament2d m_intake = m_pivot.append(new MechanismLigament2d("Intake", 3, 90));
  private MechanismLigament2d m_drivetrain = m_pivot.append(new MechanismLigament2d("Drivetrain", 1, 180));
  
  private DCMotor m_gearbox = DCMotor.getKrakenX44(1);
  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(m_gearbox, IntakePivotConstants.kPivotReduction, SingleJointedArmSim.estimateMOI(IntakePivotConstants.kIntakeLength, IntakePivotConstants.kIntakeMass), IntakePivotConstants.kIntakeLength, IntakePivotConstants.kMinAngle, IntakePivotConstants.kMaxAngle, IntakePivotConstants.kSimGravity, IntakePivotConstants.kSimStartAngle, IntakePivotConstants.kTicksPerPulse, IntakePivotConstants.kEncoderNoise);
  private DutyCycleEncoderSim m_encoderSim = new DutyCycleEncoderSim(m_encoder);



  public IntakePivot() {
    m_motor.setNeutralMode(NeutralModeValue.Brake); // setting mode to brake
    SmartDashboard.putData("Intake Pivot Sim", m_mech);
  }


  public void setSpeed(double speed){
    if (IntakePivotConstants.kUpperLimitTicks>=getEncoderTicks()&&getEncoderTicks()>=IntakePivotConstants.kStartingLimitTicks){
      m_motor.set(speed);
    }
    else{
      m_motor.set(0);
    }
  } // setting the speed method

  public double getSpeed() {
    return m_motor.get();
  } // getting the speed method

  public double getEncoderTicks() {
    return m_encoder.get();  
  } // getting the ticks

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    m_pivotSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    m_pivotSim.update(IntakePivotConstants.kLoopTime);
    m_encoderSim.set(m_pivotSim.getAngleRads());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_pivotSim.getAngleRads()));
    m_intake.setAngle(m_pivotSim.getAngleRads());
  }
}
