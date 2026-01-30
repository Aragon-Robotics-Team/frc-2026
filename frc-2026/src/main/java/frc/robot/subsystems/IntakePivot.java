// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakePivotConstants;
import frc.robot.constants.ShooterPivotConstants;

public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
  private TalonFX m_motor = new TalonFX(IntakePivotConstants.kMotorID); // creating the objects on the subsystem
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(IntakePivotConstants.kEncoderChannel);
  
  private DCMotor m_gearbox = DCMotor.getKrakenX44(1);
  private SingleJointedArmSim m_pivotSim = new SingleJointedArmSim(m_gearbox, IntakePivotConstants.kPivotReduction, SingleJointedArmSim.estimateMOI(IntakePivotConstants.kIntakeLength, IntakePivotConstants.kIntakeMass), IntakePivotConstants.kIntakeLength, IntakePivotConstants.kMinAngle, IntakePivotConstants.kMaxAngle, IntakePivotConstants.kSimGravity, IntakePivotConstants.kSimStartAngle, IntakePivotConstants.kTicksPerPulse, IntakePivotConstants.kEncoderNoise);
  private DutyCycleEncoderSim m_encoderSim = new DutyCycleEncoderSim(m_encoder);

  private Mechanism2d m_mech;
  private MechanismRoot2d m_pivot;
  private MechanismLigament2d m_intake;


  public IntakePivot(Mechanism2d mech) {
    m_mech = mech;
    m_pivot = m_mech.getRoot("Pivot", ShooterPivotConstants.kDriveTrainStart, ShooterPivotConstants.kDriveTrainHeight);
    m_intake = m_pivot.append(new MechanismLigament2d("Intake", IntakePivotConstants.kIntakeSize, Units.radiansToDegrees(m_pivotSim.getAngleRads())));

    m_motor.setNeutralMode(NeutralModeValue.Brake); // setting mode to brake
  }


  public void setSpeed(double speed){
    if (IntakePivotConstants.kUpperLimitTicks>=getEncoderTicks() && getEncoderTicks()>=IntakePivotConstants.kStartingLimitTicks){
      m_motor.set(speed);
    }
    else{
      m_motor.set(0);
      System.out.println("Out of bounds");
    }
  } // setting the speed method

  public double getSpeed() {
    return m_motor.get();
  } // getting the speed method

  public double getEncoderTicks() {
    return m_encoder.get(); // use for sim
    // return m_encoder.get() * 2.0 * Math.PI
    // use for actual intake pivot
  } // getting the ticks

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    m_pivotSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
    // m_pivotSim.setInput(0.58);
    m_pivotSim.update(IntakePivotConstants.kLoopTime);
    m_encoderSim.set(m_pivotSim.getAngleRads());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_pivotSim.getAngleRads()));
    m_intake.setAngle(- 180 - Units.radiansToDegrees(m_pivotSim.getAngleRads()));

    
    // System.out.println(m_pivotSim.getVelocityRadPerSec());
    // System.out.print("Motor");
    // System.out.println(m_motor.get() * RobotController.getBatteryVoltage());
    // System.out.print("MechAngle ");
    // System.out.println(m_intake.getAngle());
    // System.out.print("SimAngle  ");
    // System.out.println(m_pivotSim.getAngleRads());
    // System.out.println();
  }
}
