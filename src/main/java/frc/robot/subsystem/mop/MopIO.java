package frc.robot.subsystem.mop;

import edu.wpi.first.units.measure.Voltage;

public interface MopIO {

    public void setVoltage(Voltage voltage);
    public boolean getIsStuck();

    public void readPeriodic();
    public default void writePeriodic(){return;};
}
