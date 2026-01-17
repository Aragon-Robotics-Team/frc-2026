// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class MotorTesting extends SubsystemBase {
  TalonFX talonFX = new TalonFX(0);
  SparkMax sparkMax = new SparkMax(0, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public MotorTesting() {
  }

  

  @Override
  public void periodic() {
   talonFX.set(0.1);
   sparkMax.set(0.1);
  }
}
