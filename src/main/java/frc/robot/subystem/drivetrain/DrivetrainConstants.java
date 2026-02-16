package frc.robot.subystem.drivetrain;

import com.ctre.phoenix6.CANBus;

public class DrivetrainConstants {
    public static final CANBus DRIVETRAIN_CANBUS = new CANBus("Drivetrain");

    public final static int GYRO_ID = 0;

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
    }

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

    public static int getEncoderID(SwerveModule module){
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
}
