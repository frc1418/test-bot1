package frc.robot.common;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Odometry extends SubsystemBase{
    private final SwerveDriveOdometry odometry;
    // private final AHRS gyro;
    // TODO: Install navX libraries and add gyro
    private SwerveModulePosition[] modulePositions;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");
    private final NetworkTableEntry heading = table.getEntry("heading");

    private Pose2d pose;

    public Odometry(
            // AHRS gyro,
            SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions) {
        // this.gyro = gyro;
        this.odometry = odometry;
        this.modulePositions = modulePositions;
        this.pose = new Pose2d();
    }
}
