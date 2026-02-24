package frc.robot.subystem.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public interface ShooterIO {
    public AngularVelocity calculateLeaderRPM(Distance distance);
    public void setLeaderRPM(AngularVelocity targetRPM);
    public void setFollowerRPM(AngularVelocity targetRPM);

    public void readPeriodic();
    public void writePeriodic();
}
