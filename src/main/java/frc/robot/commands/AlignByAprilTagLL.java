package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.TargetSpaceOdometry;
import frc.robot.subsystems.DriveSubsystem;

public class AlignByAprilTagLL extends Command {
    PIDController speedRotController;

    DriveSubsystem swerveDrive;
    TargetSpaceOdometry odometry;

    double targetX;
    double targetY;
    double targetRot;

    boolean startedFieldCentric;

    PIDController speedController;

    public AlignByAprilTagLL(DriveSubsystem swerveDrive, double targetX, double targetY, double P, double I, double D, double targetRot) {

        this.swerveDrive = swerveDrive;
        this.odometry = swerveDrive.getTargetOdometry();
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetRot = targetRot;
    
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

        swerveDrive.drive(0,0,0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d robotPose = new Pose2d(
            new Translation2d(odometry.getPose().getX(), odometry.getPose().getY()),
            odometry.getPose().getRotation());

        Pose2d targetPose;
        targetPose = new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRot));
        
        double x;
        double y;
        double rot;

        double dx = -(targetPose.getX() - robotPose.getX());
        double dy =  targetPose.getY() - robotPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dx, dy) * 180 / Math.PI;

        rot = -speedRotController.calculate(robotPose.getRotation().getDegrees(), targetRot);

        double speed = speedController.calculate(0, distance);

        Rotation2d direction = Rotation2d.fromDegrees(angleToTarget + robotPose.getRotation().getDegrees());

        if (!speedController.atSetpoint()) {
            x = direction.getCos() * speed;
            y = direction.getSin() * speed;
        }
        else {
            x = 0;
            y = 0;
        }

        swerveDrive.drive(x, y, rot);        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        speedController.reset();
        speedRotController.reset();
        this.swerveDrive.setFieldCentric(startedFieldCentric);
        swerveDrive.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}