package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.subsystems.DriveSubsystem;

public class AlignRot extends Command {

    PIDController speedRotController;

    RobotContainer robotContainer;
    DriveSubsystem swerveDrive;
    FieldSpaceOdometry odometry;

    CommandJoystick leftJoystick;
    CommandJoystick rightJoystick;

    double targetRot;

    Optional<Alliance> ally;

    SlewRateLimiter limitX = new SlewRateLimiter(DriverConstants.maxAccel);
    SlewRateLimiter limitY = new SlewRateLimiter(DriverConstants.maxAccel);

    public AlignRot(RobotContainer robotContainer, DriveSubsystem swerveDrive, CommandJoystick leftJoystick, double targetRot) {

        ally = DriverStation.getAlliance();
        this.robotContainer = robotContainer;
        this.swerveDrive = swerveDrive;
        this.odometry = swerveDrive.getOdometry();
        this.leftJoystick = leftJoystick;
        this.targetRot = targetRot;
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                this.targetRot += 180;
            }
        }

        speedRotController = new PIDController(0.015, 0, 0.0005);
        speedRotController.enableContinuousInput(-180, 180);

        addRequirements(swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveDrive.drive(limitX.calculate(0),limitY.calculate(0),0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!ally.isPresent()) {
            ally = DriverStation.getAlliance();
            if (ally.get() == Alliance.Red) {
                this.targetRot += 180;
            }
        }

        if (swerveDrive.getCorrectRot()) {
            double rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), targetRot);

            swerveDrive.drive(
                -limitX.calculate(robotContainer.applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)),
                -limitY.calculate(robotContainer.applyDeadband(leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND)),
                rot
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        speedRotController.reset();
        swerveDrive.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}