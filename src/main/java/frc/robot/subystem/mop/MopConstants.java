package frc.robot.subystem.mop;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;


public class MopConstants {
    public static final String CANBUS = "mech";
    public static final int MOP_MOTOR_ID = 7;
    public static final Voltage Spin_VOLTAGE = Volts.of(10);
    public static final Voltage MAX_VOLTAGE = Volts.of(20);
    public static final Voltage MIN_VOLTAGE = Volts.of(5);
    public static final Voltage STARTING_VOLTAGE = Volts.of(0);
    public static final Voltage STORED_VOLTAGE = Volts.of(0);
    public static final Voltage VOLTAGE_TOLERANCE = Volts.of(0.5);

}
