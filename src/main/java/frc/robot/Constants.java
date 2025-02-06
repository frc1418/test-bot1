// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public final static class DrivetrainConstants{

    public static final double wheelPos = 0.2978;

    public static final double DRIFT_DEADBAND = 0.1;
    public static final double ROTATION_DEADBAND = 0.03;

    public static final int BACK_RIGHT_ANGLE_ID = 7;
    public static final int BACK_RIGHT_SPEED_ID = 8;
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(-wheelPos, -wheelPos);
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0.631-Math.PI/2;

    public static final int FRONT_RIGHT_ANGLE_ID = 5;
    public static final int FRONT_RIGHT_SPEED_ID = 6;
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(wheelPos, -wheelPos);
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 3.672;
    
    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final int BACK_LEFT_SPEED_ID = 2;
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(-wheelPos, wheelPos);
    public static final double BACK_LEFT_ENCODER_OFFSET = 5.339;
    
    public static final int FRONT_LEFT_ANGLE_ID = 3;
    public static final int FRONT_LEFT_SPEED_ID = 4;
    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(wheelPos, wheelPos);
    public static final double FRONT_LEFT_ENCODER_OFFSET = 3.783+Math.PI/2;
    
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT_LOC,
      FRONT_RIGHT_LOC,
      BACK_LEFT_LOC,
      BACK_RIGHT_LOC
    );
  }

  public final static class DriverConstants {
    public final static double maxAccel = 5.0;
    public final static double maxSpeedMetersPerSecond = 4.8;
    public final static double maxAngularSpeed = 2*Math.PI;
    public final static double maxCorrectiveAngularSpeed = Math.PI;
    public final static double correctiveFactor = 0.16;
    public final static double baseCorrector = 0.04;
  }

  public final static class EncoderConstants {
    public final static double ROTATIONS_TO_METERS = 0.33/8.33 * 1.17;
    public final static double TURNING_FACTOR = 2 * Math.PI;
  }
}
