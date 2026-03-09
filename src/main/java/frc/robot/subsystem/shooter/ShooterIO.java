package frc.robot.subsystem.shooter;

public interface ShooterIO {
    public void setVelocity(double velocity);

    public void readPeriodic();
    public void writePeriodic();
}
