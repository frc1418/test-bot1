package frc.robot.common;

import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Odometry extends SubsystemBase{

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/odometry");

    private final NetworkTableEntry ntX = table.getEntry("xFromBlueOrigin (m)");
    private final NetworkTableEntry ntY = table.getEntry("yFromBlueOrigin (m)");
    private final NetworkTableEntry ntAngle = table.getEntry("degreesFromRedWall (degrees)");
    private final NetworkTableEntry ntTime = table.getEntry("timeFromInit (sec)");

    private final AHRS gyro;

    private Pose2d pose;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Odometry(SwerveModulePosition[] modulePositions) {
        this.gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        this.pose = new Pose2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SWERVE_KINEMATICS, getGyroHeading(), modulePositions, pose,
            VecBuilder.fill(0.05, 0.05, 0),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getGyroHeading() {
        return gyro.getRotation2d();
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    public Rotation2d getEstimatedRot() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void update(SwerveModulePosition[] modulePositions, double lockedRot) {
        boolean rejectVision = false;
        boolean megaTag2 = true;

        poseEstimator.update(getGyroHeading(), modulePositions);

        // TODO: Incoroporate mt1 orientation into poseEstimator by averaging values over time and excluding outliers

        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate aprilTagInfo = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (aprilTagInfo == null) {
            rejectVision = true;
        }
        else if (aprilTagInfo.rawFiducials.length == 1) {
            double ambiguity = aprilTagInfo.rawFiducials[0].ambiguity;
            if (ambiguity > 0.9) {
                rejectVision = true;
            }
        }
        else if (Math.abs(gyro.getRate()) > 720) {
            rejectVision = true;
        }
        else if (aprilTagInfo.tagCount == 0) {
            rejectVision = true;
        }

        if (!rejectVision) {
            if (megaTag2) {
                ntX.setDouble(poseEstimator.getEstimatedPosition().getX());
                ntY.setDouble(poseEstimator.getEstimatedPosition().getY());
                ntAngle.setDouble(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                ntTime.setDouble(aprilTagInfo.timestampSeconds);
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                poseEstimator.addVisionMeasurement(
                    aprilTagInfo.pose,
                    aprilTagInfo.timestampSeconds
                );
            }
            else {
                LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
                if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                    poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds
                    );
                }
            }
        }
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(getGyroHeading(), modulePositions, pose);
    }
}
