package frc.robot.subystem.mop;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;

public interface mopIO {

    public void readPeriodic();
    public void writePeriodic();
    public void simulatePeriodic();


    public boolean atVoltageSetpoint();
    public boolean empty();
    public void setDriveVoltage(VoltageOut voltage);
    public double getCurrentVoltage();
    public double getCurrentMop();
    public void setTargetVoltage(Voltage voltage);
}
