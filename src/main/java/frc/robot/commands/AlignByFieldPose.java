package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.subsystems.DriveSubsystem;

public class AlignByFieldPose extends Command {
    DriveSubsystem swerveDrive;
    FieldSpaceOdometry odometry;

    double targetX;
    double targetY;
    double targetRot;
    double initialSpeedP;
    double initialRotP = 0.01;

    boolean startedFieldCentric;

    PIDController speedController;
    PIDController speedRotController;

    public AlignByFieldPose(DriveSubsystem swerveDrive, double targetX, double targetY, double targetRot, double P, double I, double D, double maxAccel) {
        this.swerveDrive = swerveDrive;
        this.odometry = swerveDrive.getOdometry();
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetRot = targetRot;
        this.initialSpeedP = P;
       
        speedController = new PIDController(P, I, D);
        speedController.setTolerance(0.05);
        speedRotController = new PIDController(0.01, 0, 0);
        speedRotController.enableContinuousInput(-180, 180);

        addRequirements(swerveDrive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startedFieldCentric = swerveDrive.getFieldCentric();
        this.swerveDrive.setFieldCentric(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (swerveDrive.getCorrectRot()) {
            Pose2d targetPose;
            targetPose = new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRot));
            
            double x;
            double y;
            double rot;
            double speed;

            double dx = targetPose.getX() - odometry.getPose().getX();
            double dy =  targetPose.getY() - odometry.getPose().getY();

            double distance = Math.hypot(dx, dy);
            double angleToTarget = Math.atan2(dy, dx) * 180 / Math.PI;
            double deltaRot = Math.abs(targetRot - odometry.getGyroHeading().getDegrees());

            if (deltaRot < 5.5) {
                speedRotController.setP(initialRotP/((deltaRot+0.5)/6));
            }
            rot = speedRotController.calculate(odometry.getGyroHeading().getDegrees(), targetRot);

            if (distance < 0.9) {
                speedController.setP(initialSpeedP/(distance+0.1));
            }
            speed = speedController.calculate(0, distance);

            Rotation2d direction = Rotation2d.fromDegrees(angleToTarget - odometry.getGyroHeading().getDegrees());

            if (!speedController.atSetpoint()) {
                x = (direction.getCos() * speed);
                y = (direction.getSin() * speed);
            }
            else {
                x = 0;
                y = 0;
            }

            swerveDrive.drive(x, y, rot);        
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        speedController.reset();
        speedController.setP(initialSpeedP);
        speedRotController.reset();
        speedRotController.setP(initialRotP);
        this.swerveDrive.setFieldCentric(startedFieldCentric);
        swerveDrive.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}