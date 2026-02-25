package frc.robot.subsystem.feeder;

import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {

    public void setVoltage(Voltage voltage);
    public void readPeriodic();
    public default void writePeriodic(){return;};
}
