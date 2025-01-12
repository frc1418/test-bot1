package frc.robot.common;

import com.studica.frc.AHRS;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;



public class Odometry extends SubsystemBase{
    private final SwerveDriveOdometry swerveOdometry;
    private final AHRS gyro;
    private SwerveModulePosition[] modulePositions;

    private Pose2d pose;

    public Odometry(SwerveModulePosition[] modulePositions) {
        this.gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        this.swerveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, getHeading(), modulePositions);
        this.modulePositions = modulePositions;
        this.pose = new Pose2d();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).unaryMinus();
    }

    public void update(SwerveModulePosition[] modulePositions) {
        swerveOdometry.update(getHeading(), modulePositions);
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        swerveOdometry.resetPosition(getHeading(), modulePositions, pose);
    }
}
