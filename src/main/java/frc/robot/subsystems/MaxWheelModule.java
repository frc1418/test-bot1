// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;;

public class MaxWheelModule extends SubsystemBase {

  private SparkMax angleMotor;
  private SparkMax speedMotor;

  private AbsoluteEncoder angleEncoder;
  private RelativeEncoder speedEncoder;
  
  private SparkClosedLoopController speedController;
  private SparkClosedLoopController angleController;

  private double targetSpeed = 0;
    
  double angleSetpoint = 0;

  public MaxWheelModule(SparkMax angleMotor, SparkMax speedMotor) {
    this.angleMotor = angleMotor;
    this.speedMotor = speedMotor;

    this.speedEncoder = speedMotor.getEncoder();
    this.angleEncoder = angleMotor.getAbsoluteEncoder();

    this.speedController = speedMotor.getClosedLoopController();
    this.angleController = angleMotor.getClosedLoopController();

    this.speedMotor.configure(Configs.MAXSwerveModule.speedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.angleMotor.configure(Configs.MAXSwerveModule.angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.speedEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(new Rotation2d(this.angleEncoder.getPosition()));

    setWheelSpeed(desiredState.speedMetersPerSecond);
    setWheelAngle(desiredState.angle.getRadians());
  }

  public void setWheelSpeed(double speed) {
    speedController.setReference(speed, ControlType.kVelocity);
  }

  public void setWheelAngle(double angle) {
    angleController.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
