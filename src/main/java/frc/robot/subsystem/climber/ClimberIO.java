package frc.robot.subsystem.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface ClimberIO {

    public void setPivotState(DoubleSolenoid.Value value);
    public void setHeight(Distance targetHeight);

    public DoubleSolenoid.Value getPivotState();

    public void readPeriodic();
    public default void writePeriodic(){return;};
}
