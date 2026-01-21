// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BridgeOutputValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private TalonFX m_turnMotor;
  private TalonFX m_driveMotor;
  private DutyCycleEncoder m_absoluteEncoder;
  private PIDController m_pid = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  private double m_absoluteEncoderOffset;
  private int m_driveReversed;
  private final VelocityVoltage m_driveVelocityInput = new VelocityVoltage(0);
  private final DutyCycleOut m_turnMotorInput = new DutyCycleOut(0);
  private String m_prefix;
  /** Creates a new SwerveModule. */

  StatusSignal<AngularVelocity> driveAngularVelocity;
  StatusSignal<Angle> drivePosition;
  StatusSignal<AngularAcceleration> driveAngularAcceleration;
  StatusSignal<BridgeOutputValue> driveBridgeOutput;
  StatusSignal<Double> driveControlSystemTotalOutput;
  StatusSignal<Double> driveControlSystemProportionalOutput;
  StatusSignal<Double> driveControlSystemReference;
  StatusSignal<Double> driveDutyCycle;
  StatusSignal<Voltage> driveVoltage;
  StatusSignal<Current> driveSupplyCurrent;
  StatusSignal<Voltage> driveSupplyVoltage;
  StatusSignal<Current> driveTorqueCurrent;
  StatusSignal<Double> drivePidDerivativeOutput;
  StatusSignal<Double> drivePidIntegralOutput;
  StatusSignal<Double> drivePidError;
  StatusSignal<Double> drivePidOutput;
  StatusSignal<Double> drivePidProportionalOutput;
  StatusSignal<Double> drivePidReference;
  StatusSignal<Double> drivePidReferenceSlope;

  StatusSignal<Angle> turnRelativeEncoderPosition;
  StatusSignal<AngularVelocity> turnAngularVelocity;
  StatusSignal<AngularAcceleration> turnAcceleration;
  StatusSignal<BridgeOutputValue> turnBridgeOutput;
  StatusSignal<Double> turnDutyCycle;
  StatusSignal<Voltage> turnVoltage;
  StatusSignal<Current> turnSupplyCurrent;
  StatusSignal<Voltage> turnSupplyVoltage;
  StatusSignal<Current> turnTorqueCurrent;

  public SwerveModule(int turnMotorID, int driveMotorID, int absoluteEncoderID, double absoluteEncoderOffset, int driveReversed, String location) {
    m_turnMotor = new TalonFX(turnMotorID);
    m_driveMotor = new TalonFX(driveMotorID);
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID, 2 * Math.PI, 0);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_driveReversed = driveReversed;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
    m_prefix = location;

    driveAngularVelocity = m_driveMotor.getVelocity();
    drivePosition = m_driveMotor.getPosition();
    driveAngularAcceleration = m_driveMotor.getAcceleration();
    driveBridgeOutput = m_driveMotor.getBridgeOutput();
    driveControlSystemTotalOutput = m_driveMotor.getClosedLoopOutput();
    driveControlSystemProportionalOutput = m_driveMotor.getClosedLoopProportionalOutput();
    driveControlSystemReference = m_driveMotor.getClosedLoopReference();
    driveDutyCycle = m_driveMotor.getDutyCycle();
    driveVoltage = m_driveMotor.getMotorVoltage();
    driveSupplyCurrent = m_driveMotor.getSupplyCurrent();
    driveSupplyVoltage = m_driveMotor.getSupplyVoltage();
    driveTorqueCurrent = m_driveMotor.getTorqueCurrent();
    drivePidDerivativeOutput = m_driveMotor.getClosedLoopDerivativeOutput();
    drivePidIntegralOutput = m_driveMotor.getClosedLoopIntegratedOutput();
    drivePidError = m_driveMotor.getClosedLoopError();
    drivePidOutput = m_driveMotor.getClosedLoopOutput();
    drivePidProportionalOutput = m_driveMotor.getClosedLoopProportionalOutput();
    drivePidReference = m_driveMotor.getClosedLoopReference();
    drivePidReferenceSlope = m_driveMotor.getClosedLoopReferenceSlope();

    turnRelativeEncoderPosition = m_turnMotor.getPosition();
    turnAngularVelocity = m_turnMotor.getVelocity();
    turnAcceleration = m_turnMotor.getAcceleration();
    turnBridgeOutput = m_turnMotor.getBridgeOutput();
    turnDutyCycle = m_turnMotor.getDutyCycle();
    turnVoltage = m_turnMotor.getMotorVoltage();
    turnSupplyCurrent = m_turnMotor.getSupplyCurrent();
    turnSupplyVoltage = m_turnMotor.getSupplyVoltage();
    turnTorqueCurrent = m_turnMotor.getTorqueCurrent();

    this.initDrivePID(m_driveMotor);
  }
  
  private int driveMotorMode = 0;{

  if (DriverStation.isTest()) {driveMotorMode = 1;}}

  private void initDrivePID(TalonFX driveCTalonFX) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    driveCTalonFX.getConfigurator().apply(slot0Configs);
  }

  public void setModuleState(SwerveModuleState state) {
    double m_currentAngle = m_absoluteEncoder.get() - m_absoluteEncoderOffset; // radians
    state.optimize(new Rotation2d(m_currentAngle));
    double m_driveMotorRotationsPerSecond = state.speedMetersPerSecond / (DriveConstants.kWheelDiameter * Math.PI) * DriveConstants.kGearRatio;
    m_driveMotor.setControl(m_driveVelocityInput.withSlot(driveMotorMode).withVelocity(m_driveMotorRotationsPerSecond));
    
    double m_error = m_pid.calculate(MathUtil.angleModulus(m_currentAngle), state.angle.getRadians());
    m_turnMotor.setControl(m_turnMotorInput.withOutput(m_error));

    //log turn pid info
    Logger.recordOutput(m_prefix+"turn/commandedAngleOfTheWheelInDegrees", state.angle);
    Logger.recordOutput(m_prefix+"turn/Pid/errorDerivative", m_pid.getErrorDerivative());
    Logger.recordOutput(m_prefix+"turn/Pid/error", m_pid.getError());
    Logger.recordOutput(m_prefix+"turn/Pid/accumulatedError", m_pid.getAccumulatedError());
    Logger.recordOutput(m_prefix+"turn/Pid/errorTolerance", m_pid.getErrorTolerance());
    Logger.recordOutput(m_prefix+"turn/Pid/errorDerivativeTolerance", m_pid.getErrorDerivativeTolerance());
    Logger.recordOutput(m_prefix+"turn/Pid/iZone", m_pid.getIZone());
    Logger.recordOutput(m_prefix+"turn/Pid/calculatedPidValue", m_error);
    Logger.recordOutput(m_prefix+"turn/Pid/kP", m_pid.getP());

    //log absolute encoder info
    Logger.recordOutput(m_prefix+"/absoluteEncoder/dutyCyclePosition", m_currentAngle / 2 * Math.PI); //0 to 1
    Logger.recordOutput(m_prefix+"/absoluteEncoder/absoluteEncoderDutyCycleOffset", m_absoluteEncoderOffset);
    Logger.recordOutput(m_prefix+"/absoluteEncoder/absoluteEncoderPositionInDegrees", m_currentAngle / Math.PI * 180);
    Logger.recordOutput(m_prefix+"/absoluteEncoder/absoluteEncoderPositionInRadians", m_currentAngle);
    Logger.recordOutput(m_prefix+"/absoluteEncoder/encoderGet", m_absoluteEncoder.get());
    Logger.recordOutput(m_prefix+"/absoluteEncoder/offset", m_absoluteEncoderOffset);
  }

  @Override
  public void periodic() {}
}
