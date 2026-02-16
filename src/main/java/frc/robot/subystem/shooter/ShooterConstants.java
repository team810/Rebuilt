package frc.robot.subystem.shooter;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {
    public static final CANBus CANBUS = new CANBus("mech");
    public static final int LEADER_MOTOR_ID = 0;
    public static final int FOLLOWER_MOTOR_ID = 1;

    public static final double LEADER_MOTOR_MAX_RPM = 5800;
    public static final double FOLLOWER_MOTOR_MAX_RPM = 5800;

}
