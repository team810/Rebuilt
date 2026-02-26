package frc.robot.subsystem.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    public void setVelocity(AngularVelocity velocity);

    public void readPeriodic();
    public void writePeriodic();
}
