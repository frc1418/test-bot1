// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AlignByFieldPose;
import frc.robot.commands.AlignByAprilTagLL;
import frc.robot.commands.AlignRot;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  CommandJoystick leftJoystick = new CommandJoystick(0);
  CommandJoystick rightJoystick = new CommandJoystick(1);
  CommandJoystick altJoystick = new CommandJoystick(2);

  private final AlignByFieldPose alignByCoralStation = new AlignByFieldPose(driveSubsystem, 16.177, 6.273, 56.7, 0.5, 0, 0.1, 3);
  private final AlignByFieldPose alignByAutoStart = new AlignByFieldPose(driveSubsystem, 17, 5, 0, 0.5, 0, 0.1, 3);
  private final AlignByAprilTagLL alignByAprilTagLL = new AlignByAprilTagLL(driveSubsystem, 0.0, -1.75, 0.3, 0.1, 0.1, 0);
  private final AlignRot alignRot = new AlignRot(this, driveSubsystem, leftJoystick, 0);
  
  public RobotContainer() {
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
     autoChooser = AutoBuilder.buildAutoChooser("Test");
     autoChooser.setDefaultOption("Default Path", null);
     SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    //Positive x moves bot forwards and positive y moves bot to the left
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      driveSubsystem.drive(
        -applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND),
        -applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND),
        -applyDeadband(rightJoystick.getX(), DrivetrainConstants.ROTATION_DEADBAND));
    }, driveSubsystem));


    leftJoystick.button(1).onTrue(driveSubsystem.setTempSlowMode(true));
    leftJoystick.button(1).onFalse(driveSubsystem.setTempSlowMode(false));
    leftJoystick.button(2).whileTrue(alignByCoralStation);
    leftJoystick.button(3).whileTrue(alignByAprilTagLL);
    leftJoystick.button(4).whileTrue(alignByAutoStart);
    leftJoystick.button(5).onTrue(driveSubsystem.toggleFieldCentric());
    leftJoystick.button(6).onTrue(driveSubsystem.toggleFastMode());

    rightJoystick.button(1).whileTrue(alignRot);
    rightJoystick.button(2).onTrue(driveSubsystem.resetFieldCentric());
    rightJoystick.button(3).whileTrue(driveSubsystem.getRotError());
    rightJoystick.button(3).onFalse(driveSubsystem.correctError());
    rightJoystick.button(4).whileTrue(driveSubsystem.turtle());
  }

  public double applyDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) 
      return 0;
    else return 
      input;
  }

  public void resetLockRot() {
    driveSubsystem.resetLockRot();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
