package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.EncoderConstants;

public final class Configs {
     public static final class MAXSwerveModule {
        public static final SparkMaxConfig speedConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        static {
            speedConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            speedConfig.encoder
                .positionConversionFactor(EncoderConstants.ROTATIONS_TO_METERS)
                .velocityConversionFactor(EncoderConstants.ROTATIONS_TO_METERS/60.0);

            // You should run SysId to get these values for each individual module
            speedConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.001, 0.0, 0.0)
                .velocityFF(0.25);

            angleConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            angleConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(EncoderConstants.TURNING_FACTOR)
                .velocityConversionFactor(EncoderConstants.TURNING_FACTOR/60.0);
            angleConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1.75,0,0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, EncoderConstants.TURNING_FACTOR);
        }
    }
}
