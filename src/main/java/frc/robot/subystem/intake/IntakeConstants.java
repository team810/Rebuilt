package frc.robot.subystem.intake;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

public class IntakeConstants {
    public static final int SOLENOID_ID = 14;
    public static final int DRIVE_MOTOR_ID = 12;

    public static final int SOLENOID_FWD_CHANNEL = 1;
    public static final int SOLENOID_REV_CHANNEL = 2;

    public static final Voltage INTAKE_VOLTAGE = Volts.of(10);
    public static final Voltage REVERSE_VOLTAGE = Volts.of(-10);
}
