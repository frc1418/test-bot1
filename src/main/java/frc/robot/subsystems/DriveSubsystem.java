package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import frc.robot.common.Odometry;

public class DriveSubsystem extends SubsystemBase {
    
    private final MaxWheelModule frontLeftWheel = new MaxWheelModule(
        DrivetrainConstants.FRONT_LEFT_SPEED_ID,
        DrivetrainConstants.FRONT_LEFT_ANGLE_ID,
        DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET
    );
      
    private final MaxWheelModule frontRightWheel = new MaxWheelModule(
        DrivetrainConstants.FRONT_RIGHT_SPEED_ID,
        DrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
        DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET
    );

    private final MaxWheelModule backLeftWheel = new MaxWheelModule(
        DrivetrainConstants.BACK_LEFT_SPEED_ID,
        DrivetrainConstants.BACK_LEFT_ANGLE_ID,
        DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET
    );

    private final MaxWheelModule backRightWheel = new MaxWheelModule(
        DrivetrainConstants.BACK_RIGHT_SPEED_ID,
        DrivetrainConstants.BACK_RIGHT_ANGLE_ID,
        DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET
    );

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/drivetrain");

    private final NetworkTableEntry ntBackRightAngleEncoder = table.getEntry("backRightAngleEncoder");
    private final NetworkTableEntry ntBackLeftAngleEncoder = table.getEntry("backLeftAngleEncoder");
    private final NetworkTableEntry ntFrontRightAngleEncoder = table.getEntry("frontRightAngleEncoder");
    private final NetworkTableEntry ntFrontLeftAngleEncoder = table.getEntry("frontLeftAngleEncoder");

    private final NetworkTableEntry ntBackRightSpeed = table.getEntry("backRightSpeed");
    private final NetworkTableEntry ntBackLeftSpeed = table.getEntry("backLeftSpeed");
    private final NetworkTableEntry ntFrontRightSpeed = table.getEntry("frontRightSpeed");
    private final NetworkTableEntry ntFrontLeftSpeed = table.getEntry("frontLeftSpeed");

    private final NetworkTableEntry ntIsFieldCentric = table.getEntry("isFieldCentric");
    private final NetworkTableEntry ntHeading = table.getEntry("heading");
    private final NetworkTableEntry ntLockedRot = table.getEntry("lockedRot");

    private final Odometry odometry;

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    //This PID controller is used to keep the robot facing the same direction when not rotating
    private PIDController rotationController = new PIDController(0.04, 0, 0); 

    private double lockedRot = 0;

    //Whatever is set here is the initial field centric parameter value
    private boolean fieldCentric = false;

    public DriveSubsystem() {
        odometry = new Odometry(getModulePositions());
        odometry.zeroHeading();
        resetLockRot();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftWheel.getPosition(),
            frontRightWheel.getPosition(),
            backLeftWheel.getPosition(),
            backRightWheel.getPosition()
        };
    }

    public void drive(double x, double y, double rot) {
        if(rot == 0) {
            rot = rotationController.calculate(odometry.getHeading().getDegrees(), lockedRot);
        }
        else {
            lockedRot = odometry.getHeading().getDegrees();
        }


        if (Math.abs(rot) > DriverConstants.ROTATION_SPEED_CAP) {
            rot = DriverConstants.ROTATION_SPEED_CAP*Math.signum(rot);
        }

        if (fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, odometry.getHeading());
        }
        else {
            speeds = new ChassisSpeeds(x, y, rot);
        }

        SwerveModuleState[] wheelStates = DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        drive(wheelStates);
    }

    public void drive(SwerveModuleState[] wheelStates) {
        frontLeftWheel.setDesiredState(wheelStates[0]);
        frontRightWheel.setDesiredState(wheelStates[1]);
        backLeftWheel.setDesiredState(wheelStates[2]);
        backRightWheel.setDesiredState(wheelStates[3]);
    }

    public void turtle() {
        frontLeftWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeftWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRightWheel.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public void resetLockRot() {
        lockedRot = odometry.getHeading().getDegrees();
    }

    public Command toggleFieldCentric() {
        return Commands.runOnce(() -> {
            fieldCentric = !fieldCentric;
        });
    }

    public Command resetFieldCentric() {
        return Commands.runOnce(() -> {
            odometry.zeroHeading();
            resetLockRot();
        });
    }

    @Override
    public void periodic() {
        odometry.update(getModulePositions());

        ntBackLeftAngleEncoder.setDouble(backLeftWheel.getPosition().angle.getRadians());
        ntBackRightAngleEncoder.setDouble(backRightWheel.getPosition().angle.getRadians());
        ntFrontLeftAngleEncoder.setDouble(frontLeftWheel.getPosition().angle.getRadians());
        ntFrontRightAngleEncoder.setDouble(frontRightWheel.getPosition().angle.getRadians());

        ntBackLeftSpeed.setDouble(backLeftWheel.getState().speedMetersPerSecond);
        ntBackRightSpeed.setDouble(backRightWheel.getState().speedMetersPerSecond);
        ntFrontLeftSpeed.setDouble(frontLeftWheel.getState().speedMetersPerSecond);
        ntFrontRightSpeed.setDouble(frontRightWheel.getState().speedMetersPerSecond);

        ntIsFieldCentric.setBoolean(fieldCentric);

        ntHeading.setDouble(odometry.getHeading().getDegrees());
        ntLockedRot.setDouble(lockedRot);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}