package frc.robot.common;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.LimelightHelpers;
import frc.robot.common.LimelightHelpers.PoseEstimate;



public class Odometry extends SubsystemBase{
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/odometry");

    private final NetworkTableEntry robot_orientation = table.getEntry("robot_orientation");

    private final AHRS gyro;

    private Pose2d pose;

    private boolean rejectVision = false;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Odometry(SwerveModulePosition[] modulePositions) {
        this.gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        this.pose = new Pose2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SWERVE_KINEMATICS, getHeading(), modulePositions, pose,
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public Rotation2d getEstimatedRot() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void update(SwerveModulePosition[] modulePositions) {
        rejectVision = false;

        poseEstimator.update(getHeading(), modulePositions);

        setRobotOrientation(gyro.getAngle(), gyro.getRate(), gyro.getPitch(), 0, gyro.getRoll(), 0);
        LimelightHelpers.PoseEstimate aprilTagInfo = LimelightHelpers.getBotPoseEstimate();

        if (aprilTagInfo == null) {
            rejectVision = true;
        }
        else if (aprilTagInfo.rawFiducials.length == 1) {
            double ambiguity = aprilTagInfo.rawFiducials[0].ambiguity;
            if (ambiguity > 0.9) {
                rejectVision = true;
            }
        }
        else if (gyro.getRate() > 720) {
            rejectVision = true;
        }
        else if (aprilTagInfo.tagCount == 0) {
            rejectVision = true;
        }


        if (!rejectVision) {
            System.out.println("X (m): " + aprilTagInfo.pose.getX());
            System.out.println("Y (m): " + aprilTagInfo.pose.getY());
            System.out.println("Rot (degrees): " + aprilTagInfo.pose.getRotation().getDegrees());
            System.out.println("Time (sec): " + aprilTagInfo.timestampSeconds);
            poseEstimator.addVisionMeasurement(aprilTagInfo.pose, aprilTagInfo.timestampSeconds);
        }
    }

    private void setRobotOrientation (
        double yaw, double yawRate, 
        double pitch, double pitchRate, 
        double roll, double rollRate) {

        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;

        robot_orientation.setDoubleArray(entries);
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(getHeading(), modulePositions, pose);
    }
}
