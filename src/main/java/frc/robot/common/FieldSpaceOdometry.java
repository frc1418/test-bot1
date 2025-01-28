package frc.robot.common;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;

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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivetrainConstants;

public class FieldSpaceOdometry {

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/odometry");

    private final NetworkTableEntry ntX = table.getEntry("xFromBlueOrigin (m)");
    private final NetworkTableEntry ntY = table.getEntry("yFromBlueOrigin (m)");
    private final NetworkTableEntry ntAngle = table.getEntry("degreesFromRedWall (degrees)");
    private final NetworkTableEntry ntGyroWorking = table.getEntry("gyroWorking");
    private final NetworkTableEntry ntCorrectRot = table.getEntry("correctRot");

    private final AHRS gyro;

    private Pose2d pose;

    private boolean correctRot = true;

    private boolean rotCorrected = false;

    private int frameCount = 0;

    private Rotation2d gyroOffset = new Rotation2d(0);

    private LimelightHelpers.PoseEstimate aprilTagInfo;

    private List<Rotation2d> errorValues = new ArrayList<>();

    private final SwerveDrivePoseEstimator poseEstimator;

    public FieldSpaceOdometry(SwerveModulePosition[] modulePositions) {
        this.gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        zeroHeading();
        this.pose = new Pose2d();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SWERVE_KINEMATICS, new Rotation2d(0), modulePositions, pose,
            VecBuilder.fill(0.05, 0.05, 0),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public void zeroHeading() {
        gyro.reset();
        gyroOffset = new Rotation2d(0);
    }

    public void getRotError() {
        if (aprilTagInfo.tagCount > 0 && gyro.isConnected()) {
            Rotation2d rotation = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose.getRotation();
            Rotation2d error = rotation.minus(getGyroHeading().minus(gyroOffset));
            errorValues.add(error);
        }
    }

    public void correctError() {
        if (errorValues.size() > 0 && gyro.isConnected()) {
            double avg = 0;
            for (int i = 0; i < errorValues.size(); i++) {
                avg += errorValues.get(i).getDegrees();
            }
            avg /= errorValues.size();
            gyroOffset = Rotation2d.fromDegrees(avg);
            System.out.println("ROT CORRECTED");
            System.out.println("Avg error: " + avg);
            correctRot = true;
            rotCorrected = true;
            errorValues.clear();
        }
    }

    public Rotation2d getGyroHeading() {
        return gyro.getRotation2d().plus(gyroOffset);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean getRotCorrected() {
        return rotCorrected;
    }

    public boolean getCorrectRot() {
        return correctRot;
    }

    public void setCorrectRot(boolean correctRot) {
        this.correctRot = correctRot;
    }

    public Rotation2d getEstimatedRot() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void update(SwerveModulePosition[] modulePositions, double lockedRot) {
        if (DriverStation.isEnabled()) {
            rotCorrected = false;
            boolean rejectVision = false;
            boolean megaTag2 = false;
            boolean megaTag1 = false;

            aprilTagInfo = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            LimelightHelpers.SetRobotOrientation("limelight", getGyroHeading().getDegrees(), 0, 0, 0, 0, 0);

            if (!gyro.isConnected()) {
                correctRot = false;
            }

            if (correctRot) {
                megaTag2 = true;
                poseEstimator.update(getGyroHeading(), modulePositions);
            }

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
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                    poseEstimator.addVisionMeasurement(
                        aprilTagInfo.pose,
                        aprilTagInfo.timestampSeconds
                    );
                }
                else if (megaTag1) {
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

            if (!correctRot) {
                getRotError();
                frameCount++;
                if (frameCount > 25) {
                    correctError();
                    frameCount = 0;
                }
            }
        }
        ntX.setDouble(poseEstimator.getEstimatedPosition().getX());
        ntY.setDouble(poseEstimator.getEstimatedPosition().getY());
        ntGyroWorking.setBoolean(gyro.isConnected());
        ntCorrectRot.setBoolean(correctRot);
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(getGyroHeading(), modulePositions, pose);
    }
}
