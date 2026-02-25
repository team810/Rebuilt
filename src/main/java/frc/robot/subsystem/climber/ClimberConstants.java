package frc.robot.subsystem.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ClimberConstants {
    public static final int MOTOR_ID = 22;

    public static final int SOLENOID_FWD_CHANNEL = 3;
    public static final int SOLENOID_REV_CHANNEL = 4;

    public static final Distance TOP_HEIGHT = Distance.ofBaseUnits(1.2, Units.Meters);
    public static final Distance BOTTOM_HEIGHT = Distance.ofBaseUnits(0, Units.Meters);
    public static final Distance CLIMB_HEIGHT = Distance.ofBaseUnits(1, Units.Meters);

}
