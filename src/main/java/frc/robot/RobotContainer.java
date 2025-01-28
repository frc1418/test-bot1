// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AlignByAprilTagGyro;
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

import static edu.wpi.first.units.Units.Degrees;


public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // I think you should use the april tags and get the pose from that. Then you can have a command for every tag
  // Need to update to 2025.2.1 to get access to the 2025 field
//  private final AprilTagFieldLayout aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
//  private Command alignByAprilTagGyro(int tagId) {
//    return new AlignByAprilTagGyro(driveSubsystem, aprilTags.getTagPose(tagId));
//  }
  private final AlignByAprilTagGyro alignByCoralStation = new AlignByAprilTagGyro(driveSubsystem, 16.177, 6.273, 90, 0.3, 0.025, 0.02);

  SlewRateLimiter limitX = new SlewRateLimiter(6);
  SlewRateLimiter limitY = new SlewRateLimiter(6);

  private RobotBase robot;

  public RobotContainer(RobotBase robot) {
    this.robot = robot;
    configureBindings();
  }


  private void configureBindings() {
    // You can use the CommandJoystick class to more easily bind commands to joystick buttons
    CommandJoystick commandLeftJoystick = new CommandJoystick((0));
    commandLeftJoystick.button(1).whileTrue(alignByCoralStation);

    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick alJoystick = new Joystick(2);

    JoystickButton fieldCentricButton = new JoystickButton(leftJoystick, 1);
    JoystickButton resetFieldCentricButton = new JoystickButton(leftJoystick, 2);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);
    JoystickButton alignByCoralStationButton = new JoystickButton(rightJoystick, 2);
    JoystickButton fixHeadingButton = new JoystickButton(rightJoystick, 3);


    //Positive x moves bot forwards and positive y moves bot to the left
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
        // Why do you need to check if teleop is enabled here?
      if (robot.isTeleopEnabled()){
        if (driveSubsystem.getFieldCentric()) {
          driveSubsystem.drive(
            -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)),
            -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND)),
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND));
        }
        else {
          driveSubsystem.drive(
            -limitX.calculate(applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)),
            -limitY.calculate(applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND)),
            applyDeadband(-rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND));
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
