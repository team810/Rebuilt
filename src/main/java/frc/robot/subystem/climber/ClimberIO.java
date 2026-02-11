package frc.robot.subystem.climber;

import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {
    public void setVoltage(Voltage voltage);
    public boolean extended();
    public void readPeriodic();
    public default void writePeriodic(){return;};
    public default void simulationPeriodic(){return;};
}
