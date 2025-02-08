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

    public static final int FRONT_LEFT_ANGLE_ID = 3;
    public static final int FRONT_LEFT_SPEED_ID = 4;
    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(wheelPos, wheelPos);
    public static final double FRONT_LEFT_ENCODER_OFFSET = 3.783+Math.PI/2;
    public static final double FRONT_LEFT_P = 0.00040465;
    public static final double FRONT_LEFT_D = 0;
    public static final double FRONT_LEFT_KS = 0.17025;
    public static final double FRONT_LEFT_KV = 2.7288;
    public static final double FRONT_LEFT_KA = 0.34097;

    public static final int FRONT_RIGHT_ANGLE_ID = 5;
    public static final int FRONT_RIGHT_SPEED_ID = 6;
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(wheelPos, -wheelPos);
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 3.672;
    public static final double FRONT_RIGHT_P = 0.00080684;
    public static final double FRONT_RIGHT_D = 0;
    public static final double FRONT_RIGHT_KS = 0.22297;
    public static final double FRONT_RIGHT_KV = 2.6277;
    public static final double FRONT_RIGHT_KA = 0.38782;
    

    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final int BACK_LEFT_SPEED_ID = 2;
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(-wheelPos, wheelPos);
    public static final double BACK_LEFT_ENCODER_OFFSET = 5.339;
    public static final double BACK_LEFT_P = 0.0012982;
    public static final double BACK_LEFT_D = 0;
    public static final double BACK_LEFT_KS = 0.22297;
    public static final double BACK_LEFT_KV = 2.5534;
    public static final double BACK_LEFT_KA = 0.36435;

    public static final int BACK_RIGHT_ANGLE_ID = 7;
    public static final int BACK_RIGHT_SPEED_ID = 8;
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(-wheelPos, -wheelPos);
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0.631-Math.PI/2;
    public static final double BACK_RIGHT_P = 0.00011772;
    public static final double BACK_RIGHT_D = 0;
    public static final double BACK_RIGHT_KS = 0.21364;
    public static final double BACK_RIGHT_KV = 2.6634;
    public static final double BACK_RIGHT_KA = 0.19487;
    
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT_LOC,
      FRONT_RIGHT_LOC,
      BACK_LEFT_LOC,
      BACK_RIGHT_LOC
    );
  }

  public final static class DriverConstants {
    public final static double maxAccel = 1.5;
    public final static double maxSpeedMetersPerSecond = 4.8;
    public final static double maxAngularAccel = 0.075;
    public final static double maxAngularSpeed = 2*Math.PI;
    public final static double maxCorrectiveAngularSpeed = Math.PI;
    public final static double correctiveFactor = 0.16;
    public final static double baseCorrector = 0.04;
  }

  public final static class EncoderConstants {
    /*
     * Conversion factor from motor rotations to meters
     * 1.5 is the wheel radius in inches
     * 0.0254 converts the radius to meters
     * 2 Pi converts to radius to circumference
     * 5.08 is the gear ratio between the motor and the wheel
     */
    public final static double ROTATIONS_TO_METERS = 1.5*0.0254*2*Math.PI/5.08; 
    public final static double TURNING_FACTOR = 2 * Math.PI;
  }
}
