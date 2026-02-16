package frc.robot.subystem.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Drivetrain {
    //one instance of Drivetrain
    private static final Drivetrain instance = new Drivetrain();

    private final Pigeon2 gyro;

    private Drivetrain() {
        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.DRIVETRAIN_CANBUS);
    }

    public static Drivetrain getInstance(){
        return instance;
    }
}
