package frc.robot.subystem.climber;

import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;

public class ClimberConstatnts {
    public static final String CAN_BUS = "mech";
    public static final int MOTOR_ID = 5;
    public static final Voltage EXTENDED_VOLTAGE = Voltage.ofBaseUnits(7, Volts);
    public static final Voltage RETRACTED_VOLTAGE = Voltage.ofBaseUnits(1, Volts);
}
