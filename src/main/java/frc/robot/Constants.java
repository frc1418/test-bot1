// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static class DrivetrainConstants{

    public static final double DRIFT_DEADBAND = 0.1;
    public static final double ROTATION_DEADBAND = 0.002;

    public static final int BACK_RIGHT_ANGLE_ID = 7;
    public static final int BACK_RIGHT_SPEED_ID = 8;
    public static final AnalogInput BACK_RIGHT_ENCODER = new AnalogInput(3);
    public static final Translation2d BACK_RIGHT_LOC = new Translation2d(-0.238125, -0.238125);
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0.845;

    public static final int FRONT_RIGHT_ANGLE_ID = 5;
    public static final int FRONT_RIGHT_SPEED_ID = 6;
    public static final AnalogInput FRONT_RIGHT_ENCODER = new AnalogInput(0);
    public static final Translation2d FRONT_RIGHT_LOC = new Translation2d(0.238125, -0.238125);
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.586;
    
    public static final int BACK_LEFT_ANGLE_ID = 1;
    public static final int BACK_LEFT_SPEED_ID = 2;
    public static final AnalogInput BACK_LEFT_ENCODER = new AnalogInput(2);
    public static final Translation2d BACK_LEFT_LOC = new Translation2d(-0.238125, 0.238125);
    public static final double BACK_LEFT_ENCODER_OFFSET = 0.654+0.25;
    
    public static final int FRONT_LEFT_ANGLE_ID = 3;
    public static final int FRONT_LEFT_SPEED_ID = 4;
    public static final AnalogInput FRONT_LEFT_ENCODER = new AnalogInput(1);
    public static final Translation2d FRONT_LEFT_LOC = new Translation2d(0.238125, 0.238125);
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0.852;
    
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT_LOC,
      FRONT_RIGHT_LOC,
      BACK_LEFT_LOC,
      BACK_RIGHT_LOC);
  }

  public final static class DriverConstants {
    public final static double speedMultiplier = 3;
    public final static double angleMultiplier = 4;
    public static final double ROTATION_SPEED_CAP = 5;
  }

  public final static class EncoderConstants {
    public final static double ROTATIONS_TO_METERS = 0.33/8.33 * 1.17;
    public final static double TURNING_FACTOR = 2*Math.PI;
  }

  public enum LimelightDirections {
    SPEAKER_SIDE(0), BLUE_SOURCE_SIDE(120), RED_SOURCE_SIDE(-120), BLUE_AMP_SIDE(-90), RED_AMP_SIDE(90);

    private int angle;

    LimelightDirections(int angle){
        this.angle = angle;
    }

    public int angle(){
        return angle;
    }

  }
}
