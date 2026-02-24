package frc.robot.subystem.feeder;

import com.ctre.phoenix6.CANBus;

public class FeederConstants {
    public static final int FEEDER_MOTOR_ID = 2;
    public static final double FEEDER_MOTOR_MAX_RPM = 5800;
    public static final CANBus CANBUS = new CANBus("mech");

}
