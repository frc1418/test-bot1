package frc.robot.subsystems;

import java.util.Optional;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.FieldSpaceOdometry;
import frc.robot.common.TargetSpaceOdometry;

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
    private final NetworkTableEntry ntEstimatedRot = table.getEntry("estimatedRot");

    private final FieldSpaceOdometry fieldOdometry;

    private final TargetSpaceOdometry targetOdometry;

    private ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    //This PID controller is used to keep the robot facing the same direction when not rotating
    private PIDController rotationController = new PIDController(DriverConstants.baseCorrector, 0, 0); 

    private double lockedRot;

    private Optional<Alliance> ally;

    private boolean fieldCentric = true;

    SysIdRoutine routine;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public DriveSubsystem() {
        ally = DriverStation.getAlliance();
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(voltage -> {
                backLeftWheel.setVoltage(voltage);
                backRightWheel.setVoltage(voltage);
                frontLeftWheel.setVoltage(voltage);
                frontRightWheel.setVoltage(voltage);
            }, 
            log -> {
                log.motor("backLeft")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backLeftWheel.getMotor().get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backLeftWheel.getMotor().getEncoder().getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backLeftWheel.getMotor().getEncoder().getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("backRight")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            backRightWheel.getMotor().get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(backRightWheel.getMotor().getEncoder().getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(backRightWheel.getMotor().getEncoder().getVelocity(), MetersPerSecond));

                log.motor("frontLeft")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        frontLeftWheel.getMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(frontLeftWheel.getMotor().getEncoder().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(frontLeftWheel.getMotor().getEncoder().getVelocity(), MetersPerSecond));
                    
                log.motor("frontRight")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        frontRightWheel.getMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(frontRightWheel.getMotor().getEncoder().getPosition(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(frontRightWheel.getMotor().getEncoder().getVelocity(), MetersPerSecond));
            }, 
            this));
        fieldOdometry = new FieldSpaceOdometry(getModulePositions(), ally);
        targetOdometry = new TargetSpaceOdometry(getModulePositions(), fieldOdometry);
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
        double xSpeed = x*DriverConstants.maxSpeedMetersPerSecond;
        double ySpeed = y*DriverConstants.maxSpeedMetersPerSecond;
        double rotSpeed = rot*DriverConstants.maxAngularSpeed;

        if (fieldOdometry.isRotJustCorrected()) {
            resetLockRot();
        }
        
        if (!ally.isPresent()) {
            ally = DriverStation.getAlliance();
        }

        if(rotSpeed == 0 && fieldOdometry.isCorrectRot() && Math.abs(fieldOdometry.getGyroHeading().getDegrees() - lockedRot) < 180) {
            if (Math.hypot(x, y) > 0.25) {
                rotationController.setP(Math.hypot(x,y)*DriverConstants.correctiveFactor);
            }
            else {
                rotationController.setP(DriverConstants.baseCorrector);
            }
            rotSpeed = rotationController.calculate(fieldOdometry.getGyroHeading().getDegrees(), lockedRot);
            if (Math.abs(rotSpeed) > DriverConstants.maxCorrectiveAngularSpeed) {
                rotSpeed = DriverConstants.maxCorrectiveAngularSpeed*Math.signum(rotSpeed);
            }
        }
        else {
            resetLockRot();
        }


        if (fieldCentric && fieldOdometry.isCorrectRot()) {
            if (ally.get() == Alliance.Red) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, new Rotation2d(fieldOdometry.getGyroHeading().getRadians()+Math.PI));
            }
            else {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, fieldOdometry.getGyroHeading());
            }
        }
        else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        SwerveModuleState[] wheelStates = DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelStates, DriverConstants.maxSpeedMetersPerSecond);
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

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public FieldSpaceOdometry getOdometry() {
        return fieldOdometry;
    }

    public TargetSpaceOdometry getTargetOdometry() {
        return targetOdometry;
    }

    public double getLockedRot() {
        return lockedRot;
    }

    public boolean getCorrectRot() {
        return fieldOdometry.isCorrectRot();
    }

    public void resetLockRot() {
        lockedRot = fieldOdometry.getGyroHeading().getDegrees();
    }

    public Command getRotError() {
        return Commands.runOnce(() -> {
            fieldOdometry.findRotError();
        });
    }

    public Command correctError() {
        return Commands.runOnce(() -> {
            fieldOdometry.correctError();
        });
    }

    public Command toggleFieldCentric() {
        return Commands.runOnce(() -> {
            fieldCentric = !fieldCentric;
        });
    }

    public Command resetFieldCentric() {
        return Commands.runOnce(() -> {
            fieldOdometry.zeroHeading();
            resetLockRot();
            fieldOdometry.setCorrectRot(true);
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
      }
    
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
      }

    @Override
    public void periodic() {
        fieldOdometry.update(getModulePositions(), lockedRot);
        targetOdometry.update(getModulePositions());

        ntBackLeftAngleEncoder.setDouble(backLeftWheel.getPosition().angle.getRadians());
        ntBackRightAngleEncoder.setDouble(backRightWheel.getPosition().angle.getRadians());
        ntFrontLeftAngleEncoder.setDouble(frontLeftWheel.getPosition().angle.getRadians());
        ntFrontRightAngleEncoder.setDouble(frontRightWheel.getPosition().angle.getRadians());

        ntBackLeftSpeed.setDouble(backLeftWheel.getState().speedMetersPerSecond);
        ntBackRightSpeed.setDouble(backRightWheel.getState().speedMetersPerSecond);
        ntFrontLeftSpeed.setDouble(frontLeftWheel.getState().speedMetersPerSecond);
        ntFrontRightSpeed.setDouble(frontRightWheel.getState().speedMetersPerSecond);

        ntIsFieldCentric.setBoolean(fieldCentric);

        ntHeading.setDouble(fieldOdometry.getGyroHeading().getDegrees());
        ntLockedRot.setDouble(lockedRot);
        ntEstimatedRot.setDouble(fieldOdometry.getEstimatedRot().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}