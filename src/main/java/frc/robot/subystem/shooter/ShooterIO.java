package frc.robot.subystem.shooter;

public interface ShooterIO {
    public void setLeaderTargetRPM(double targetRPM);
    public void setFollowerTargetRPM(double targetRPM);

    public void readPeriodic();
    public void writePeriodic();
}
