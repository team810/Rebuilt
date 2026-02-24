package frc.robot.subystem.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface IntakeIO {

    public void setPivotState(DoubleSolenoid.Value value);
    public void setVoltage(Voltage voltage);


    public DoubleSolenoid.Value getPivotState();


    public void readPeriodic();
    public default void writePeriodic(){return;};
    public default void simulationPeriodic(){return;};
}
