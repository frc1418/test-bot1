// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AlignByAprilTagGyro;
import frc.robot.commands.AlignRot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  SlewRateLimiter limitX = new SlewRateLimiter(DriverConstants.maxAccel);
  SlewRateLimiter limitY = new SlewRateLimiter(DriverConstants.maxAccel);

  Joystick leftJoystick = new Joystick(0);
  Joystick rightJoystick = new Joystick(1);
  Joystick alJoystick = new Joystick(2);

  private final AlignByAprilTagGyro alignByCoralStation = new AlignByAprilTagGyro(driveSubsystem, 16.177, 6.273, 90, 0.7, 0.1, 0.1, 3);
  private final AlignRot alignRot = new AlignRot(this, driveSubsystem, leftJoystick, 0);

  private RobotBase robot;
  
  public RobotContainer(RobotBase robot) {
    this.robot = robot;
    configureBindings();
  }

  private void configureBindings() {
    JoystickButton fieldCentricButton = new JoystickButton(leftJoystick, 1);
    JoystickButton resetFieldCentricButton = new JoystickButton(leftJoystick, 2);
    JoystickButton alignRotButton = new JoystickButton(rightJoystick, 1);
    JoystickButton alignByCoralStationButton = new JoystickButton(rightJoystick, 2);
    JoystickButton fixHeadingButton = new JoystickButton(rightJoystick, 3);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 4);



    //Positive x moves bot forwards and positive y moves bot to the left
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      if (robot.isTeleopEnabled()){
        driveSubsystem.drive(
          -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)),
          -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND)),
          applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND));
      }
      else 
      {
        driveSubsystem.drive(0,0,0);
      }
      
    }, driveSubsystem));

    turtleButton.whileTrue(new RunCommand(() -> {
      driveSubsystem.turtle();
    }, driveSubsystem));

    alignRotButton.whileTrue(alignRot);
    alignByCoralStationButton.whileTrue(alignByCoralStation);
    fieldCentricButton.onTrue(driveSubsystem.toggleFieldCentric());
    resetFieldCentricButton.onTrue(driveSubsystem.resetFieldCentric());
    fixHeadingButton.whileTrue(driveSubsystem.getRotError());
    fixHeadingButton.onFalse(driveSubsystem.correctError());
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
