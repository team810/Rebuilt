package frc.robot.subystem.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public interface FeederIO {
    public AngularVelocity calculateFeederRPM(Distance distance);

    public void readPeriodic();
    public void writePeriodic();
}
