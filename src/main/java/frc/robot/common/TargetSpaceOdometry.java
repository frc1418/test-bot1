package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivetrainConstants;

public class TargetSpaceOdometry {
    private final SwerveDriveOdometry odometry;
    private final FieldSpaceOdometry fieldSpaceOdometry;
    private SwerveModulePosition[] modulePositions;
    private Rotation2d oldGyro;

    public TargetSpaceOdometry(SwerveModulePosition[] modulePositions, FieldSpaceOdometry fieldSpaceOdometry) {
        this.fieldSpaceOdometry = fieldSpaceOdometry;
        this.odometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, getGyroHeading(), modulePositions);
        this.modulePositions = modulePositions;
        oldGyro = this.fieldSpaceOdometry.getGyroHeading();
    }

    public void update(SwerveModulePosition[] newPositions) {
        if (LimelightHelpers.getTV("limelight") && DriverStation.isTeleop()){
            double[] botPos_targetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
            double[] camPos_targetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight");

            if (botPos_targetSpace.length > 0 && camPos_targetSpace.length > 0) {
                reset(
                    new Pose2d(new Translation2d(botPos_targetSpace[0], botPos_targetSpace[2]), new Rotation2d(camPos_targetSpace[4]*Math.PI/180))
                );
            }
        }
        else {
            Rotation2d deltaGyro = fieldSpaceOdometry.getGyroHeading().minus(oldGyro);
            odometry.update(odometry.getPoseMeters().getRotation().minus(deltaGyro), newPositions);
        }

        oldGyro = fieldSpaceOdometry.getGyroHeading();
        modulePositions = newPositions;
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroHeading() {
        return fieldSpaceOdometry.getGyroHeading();
    }
}