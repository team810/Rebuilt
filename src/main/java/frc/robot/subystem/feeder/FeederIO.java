package frc.robot.subystem.feeder;

import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {
    public void setVoltage(Voltage voltage);
    public void readPeriodic();
    public void writePeriodic();
}
