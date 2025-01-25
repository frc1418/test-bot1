package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class AlignByAprilTagGyro extends Command {

    PIDController speedXController;
    PIDController speedYController;

    PIDController speedRotController;

    DriveSubsystem swerveDrive;
    Odometry odometry;

    double targetX;
    double targetY;

    double targetRot;
    double approachAngle;

    boolean startedFieldCentric;

    PIDController speedController;

    SlewRateLimiter limitX = new SlewRateLimiter(5);
    SlewRateLimiter limitY = new SlewRateLimiter(5);

    public AlignByAprilTagGyro(DriveSubsystem swerveDrive, double targetX, double targetY, double targetRot, double P, double I, double D) {

        this.swerveDrive = swerveDrive;
        this.odometry = swerveDrive.getOdometry();
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetRot = targetRot;
       
        speedController = new PIDController(P, I, D);
        speedRotController = new PIDController(0.01, 0.0004, 0);
        speedRotController.enableContinuousInput(-180, 180);

        addRequirements(swerveDrive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startedFieldCentric = swerveDrive.getFieldCentric();
        this.swerveDrive.setFieldCentric(false);

        swerveDrive.drive(limitX.calculate(0),limitY.calculate(0),0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Pose2d robotPose = new Pose2d(
            new Translation2d(odometry.getPose().getX(), odometry.getPose().getY()),
            odometry.getPose().getRotation());

        System.out.println("Robot X: " + robotPose.getX());
        System.out.println("Robot Y: " + robotPose.getY());

        Pose2d targetPose;
        targetPose = new Pose2d(new Translation2d(targetX, targetY), Rotation2d.fromDegrees(targetRot));
        
        double x;
        double y;
        double rot;

        double dx = targetPose.getX() - robotPose.getX();
        double dy =  targetPose.getY() - robotPose.getY();

        double distance = Math.hypot(dx, dy);
        double angleToTarget = Math.atan2(dy, dx) * 180 / Math.PI;

        rot = speedRotController.calculate(odometry.getPose().getRotation().getDegrees(), targetRot);

        double speed = speedController.calculate(0, distance);

        Rotation2d direction = Rotation2d.fromDegrees(angleToTarget - odometry.getGyroHeading().getDegrees());
        System.out.println("Direction: " + direction.getDegrees());
        System.out.println("Speed: " + speed);

        x = (direction.getCos() * speed);
        y = (direction.getSin() * speed);

        System.out.println("X: " + x);
        System.out.println("Y: " + y);
        System.out.println("Rot: " + rot);

        swerveDrive.drive(limitX.calculate(x), limitY.calculate(y), rot);        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        this.swerveDrive.setFieldCentric(startedFieldCentric);
        swerveDrive.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}