// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.BridgeOutputValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  private TalonFX m_turnMotor;
  private TalonFX m_driveMotor;
  private DutyCycleEncoder m_absoluteEncoder;
  private PIDController m_pidTurn = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
  private double m_absoluteEncoderOffset;
  private int m_driveReversed;
  private final VelocityVoltage m_driveVelocityInput = new VelocityVoltage(0);
  private final DutyCycleOut m_turnMotorInput = new DutyCycleOut(0);
  private String m_prefix;
  private double m_currentAngle;
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
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID, 2 * Math.PI, absoluteEncoderOffset);
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_driveReversed = driveReversed;
    m_pidTurn.enableContinuousInput(-Math.PI, Math.PI);
    m_prefix = location;

    //drive values
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

    //turn values
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

  private void initDrivePID(TalonFX driveCTalonFX) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    driveCTalonFX.getConfigurator().apply(slot0Configs);
  }

  public double getCurrentAngle() {
    m_currentAngle = m_absoluteEncoder.get() - m_absoluteEncoderOffset;
    return m_currentAngle;
  }

  public void setModuleState(SwerveModuleState state) {
    getCurrentAngle(); // radians
    state.optimize(new Rotation2d(m_currentAngle));
    double m_driveMotorRotationsPerSecond = state.speedMetersPerSecond / (DriveConstants.kWheelDiameter * Math.PI) * DriveConstants.kGearRatio;

    m_driveMotor.setControl(m_driveVelocityInput.withSlot(0).withVelocity(m_driveMotorRotationsPerSecond * m_driveReversed));
    
    double m_error = m_pidTurn.calculate(MathUtil.angleModulus(m_currentAngle), state.angle.getRadians());
    m_turnMotor.setControl(m_turnMotorInput.withOutput(m_error));

    logData(state, m_currentAngle, m_error, m_driveMotorRotationsPerSecond);
  }

  public double getDrivePosition() {
    return m_driveMotor.getPosition().refresh().getValueAsDouble();
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getCurrentAngle()));
  }

  private void logData(SwerveModuleState moduleState, double currentAngle, double error, double motorRotationsPerSecond) {
    BaseStatusSignal.refreshAll(driveAngularAcceleration,
      driveAngularVelocity,
      driveBridgeOutput,
      driveControlSystemProportionalOutput,
      driveControlSystemReference,
      driveControlSystemTotalOutput,
      driveDutyCycle,
      drivePidDerivativeOutput,
      drivePidError,
      drivePidIntegralOutput,
      drivePidOutput,
      drivePidProportionalOutput,
      drivePidReference,
      drivePidReferenceSlope,
      drivePosition,
      driveSupplyCurrent,
      driveSupplyVoltage,
      driveTorqueCurrent,
      driveVoltage,
      turnAcceleration,
      turnAngularVelocity,
      turnBridgeOutput,
      turnDutyCycle,
      turnRelativeEncoderPosition,
      turnSupplyCurrent,
      turnSupplyVoltage,
      turnVoltage,
      turnTorqueCurrent);

    //log turn pid info
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/errorDerivative", m_pidTurn.getErrorDerivative());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/error", m_pidTurn.getError());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/accumulatedError", m_pidTurn.getAccumulatedError());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/errorTolerance", m_pidTurn.getErrorTolerance());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/errorDerivativeTolerance", m_pidTurn.getErrorDerivativeTolerance());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/iZone", m_pidTurn.getIZone());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/calculatedPidValue", error);
    Logger.recordOutput("swerve/"+m_prefix+"/turn/Pid/kP", m_pidTurn.getP());

    //log absolute encoder info
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/dutyCyclePosition", currentAngle / 2 * Math.PI); //0 to 1
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/absoluteEncoderDutyCycleOffset", m_absoluteEncoderOffset);
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/absoluteEncoderPositionInDegrees", currentAngle / Math.PI * 180);
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/absoluteEncoderPositionInRadians", currentAngle);
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/encoderGet"+" "+m_prefix, m_absoluteEncoder.get());
    Logger.recordOutput("swerve/"+m_prefix+"/absoluteEncoder/offset", m_absoluteEncoderOffset);

    //log turn info
    Logger.recordOutput("swerve/"+m_prefix+"/turn/commandedAngleOfTheWheelInDegrees", moduleState.angle);
    Logger.recordOutput("swerve/"+m_prefix+"/turn/relativeEncoderPosition", turnRelativeEncoderPosition.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/angularVelocity", turnAngularVelocity.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/acceleration", turnAcceleration.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/bridgeOutput", turnBridgeOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/dutyCycle", turnDutyCycle.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/voltage", turnVoltage.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/supplyCurrent", turnSupplyCurrent.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/supplyVoltage", turnSupplyCurrent.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/turn/torqueCurrent", turnTorqueCurrent.getValueAsDouble());
    
    //log drive info
    double angularVelocityRotationsPerSecond = driveAngularVelocity.getValue().in(RotationsPerSecond);
    double angularVelocityRadiansPerSecond = Units.rotationsToRadians(driveAngularVelocity.getValueAsDouble());
    double foo = Units.radiansPerSecondToRotationsPerMinute(angularVelocityRadiansPerSecond)*60;
    Logger.recordOutput("swerve/"+m_prefix+"/drive/angularVelocity", driveAngularVelocity.getValueAsDouble());
    //System.out.println(driveAngularVelocity.getValue().baseUnit().name());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/angularVelocity", driveAngularVelocity.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/commandedMotorRotationsPerSecond", motorRotationsPerSecond);
    Logger.recordOutput("swerve/"+m_prefix+"/drive/angularVelocity", driveAngularVelocity.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/position", drivePosition.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/angularAcceleration", driveAngularAcceleration.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/bridgeOutput", driveBridgeOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/controlSystemTotalOutput", driveControlSystemTotalOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/controlSystemProportionalOutput", driveControlSystemProportionalOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/controlSystemReference", driveControlSystemReference.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/dutyCycle", driveDutyCycle.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/voltage", driveVoltage.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/supplyCurrent", driveSupplyCurrent.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/supplyVoltage", driveSupplyVoltage.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/torqueCurrent", driveTorqueCurrent.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/speedMetersPerSecond", moduleState.speedMetersPerSecond);

    //drive pid info
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/derivativeOutput", drivePidDerivativeOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/integralOutput", drivePidIntegralOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/pidError", drivePidError.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/pidOutput", drivePidOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/proportionalOutput", drivePidProportionalOutput.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/reference", drivePidReference.getValueAsDouble());
    Logger.recordOutput("swerve/"+m_prefix+"/drive/Pid/referenceSlope", drivePidReferenceSlope.getValueAsDouble());
  }

  @Override
  public void periodic() {}
}
