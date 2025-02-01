// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class MaxWheelModule extends SubsystemBase {

  private SparkMax speedMotor;
  private SparkMax angleMotor;

  private SparkMaxConfig speedConfig;
  private SparkMaxConfig angleConfig;
  
  private RelativeEncoder speedEncoder;
  private AbsoluteEncoder angleEncoder;
  
  private SparkClosedLoopController speedController;
  private SparkClosedLoopController angleController;

  private SimpleMotorFeedforward speedFeedforward;

  private double chassisAngularOffset = 0;

  public MaxWheelModule(int speedMotorID, int angleMotorID, double chassisAngularOffset, double p, double d, double ks, double kv, double ka) {
    this.speedMotor = new SparkMax(speedMotorID, MotorType.kBrushless);
    this.angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);

    this.speedConfig = Configs.MAXSwerveModule.getSpeedConfig();
    this.angleConfig = Configs.MAXSwerveModule.getAngleConfig();

    this.speedConfig.closedLoop.pid(p, 0, d);
    this.speedFeedforward = new SimpleMotorFeedforward(ks, kv, ka);

    this.speedEncoder = speedMotor.getEncoder();
    this.angleEncoder = angleMotor.getAbsoluteEncoder();

    this.speedController = speedMotor.getClosedLoopController();
    this.angleController = angleMotor.getClosedLoopController();

    this.speedMotor.configure(this.speedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.angleMotor.configure(this.angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.speedEncoder.setPosition(0);
    this.chassisAngularOffset = chassisAngularOffset;
    
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        speedEncoder.getPosition(),
        new Rotation2d(angleEncoder.getPosition() - chassisAngularOffset));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        speedEncoder.getVelocity(),
        new Rotation2d(angleEncoder.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    desiredState.optimize(new Rotation2d(this.angleEncoder.getPosition()));

    setWheelSpeed(desiredState.speedMetersPerSecond);
    setWheelAngle(desiredState.angle.getRadians());
  }

  public void setWheelSpeed(double speed) {
    System.out.println(speed);
    if (Math.abs(speed) < .1)
      speed = 0;
    speedController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, speedFeedforward.calculate(speed));
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
