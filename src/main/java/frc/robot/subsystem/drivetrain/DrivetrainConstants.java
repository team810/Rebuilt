package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public final static CANBus DRIVETRAIN_CANBUS = new CANBus("Drivetrain");

    public final static int GYRO_ID = 13;

    public final static Boolean USING_VISION = true;

    public final static String LIMELIGHT_SHOOTER = "limelight-shooter";

    public final static double THETA_KP = 0;
    public final static double THETA_KI = 0;
    public final static double THETA_KD = 0;

    public final static double LINEAR_KP = 0;
    public final static double LINEAR_KI = 0;
    public final static double LINEAR_KD = 0;

    public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(21); // measure of FL wheel to FR wheel or BL wheel to BR wheel
    public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(21);
    public final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
        // Front right
        new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
        // Back left
        new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
        // Back right
        new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0)
    );

    public static int getDriveID(SwerveModule module) {
        switch (module) {
            case FrontLeft -> {
                return 5;
            }
            case FrontRight -> {
                return 6;
            }
            case BackLeft -> {
                return 7;
            }
            case BackRight -> {
                return 8;
            }
        }
        return 0;
    }

    public static int getSteerID(SwerveModule module) {
        switch (module) {
            case FrontLeft -> {
                return 1;
            }
            case FrontRight -> {
                return 2;
            }
            case BackLeft -> {
                return 3;
            }
            case BackRight -> {
                return 4;
            }
        }
        return 0;

    };

    public static int getEncoderID(SwerveModule module) {
        switch (module) {
            case FrontLeft -> {
                return 9;
            }
            case FrontRight -> {
                return 10;
            }
            case BackLeft -> {
                return 11;
            }
            case BackRight -> {
                return 12;
            }
        }
        return 0;
    }


    public static TalonFXConfiguration getDriveConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.Slot0.kV = .1241;
        config.Slot0.kG = 0;
        config.Slot0.kS = 0;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Audio.BeepOnConfig = true;
        config.Audio.BeepOnBoot = true;

        return config;
    }

    public static TalonFXConfiguration getSteerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.Slot0.kV = .1241;
        config.Slot0.kG = 0;
        config.Slot0.kS = 0;
        config.Slot0.kP = 1.9;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        config.Audio.BeepOnConfig = true;
        config.Audio.BeepOnBoot = true;

        return config;
    }
}
