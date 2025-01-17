// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  SlewRateLimiter limitX = new SlewRateLimiter(6);
  SlewRateLimiter limitY = new SlewRateLimiter(6);

  private RobotBase robot;
  
  public RobotContainer(RobotBase robot) {
    this.robot = robot;
    configureBindings();
  }


  private void configureBindings() {
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick alJoystick = new Joystick(2);

    JoystickButton fieldCentricButton = new JoystickButton(leftJoystick, 1);
    JoystickButton resetFieldCentricButton = new JoystickButton(leftJoystick, 2);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);

    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        if (driveSubsystem.getFieldCentric()) {
          driveSubsystem.drive(
            -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
        }
        else {
          driveSubsystem.drive(
            -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND))*DriverConstants.speedMultiplier,
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND)*DriverConstants.angleMultiplier);
        }
      }
      else 
      {
        driveSubsystem.drive(0,0,0);
      }
      
    }, driveSubsystem));

    turtleButton.whileTrue(new RunCommand(() -> {
      driveSubsystem.turtle();
    }, driveSubsystem));

    fieldCentricButton.onTrue(driveSubsystem.toggleFieldCentric());
    resetFieldCentricButton.onTrue(driveSubsystem.resetFieldCentric());
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
