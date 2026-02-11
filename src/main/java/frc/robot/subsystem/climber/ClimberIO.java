package frc.robot.subsystem.climber;

public interface ClimberIO {
    void setVoltage(double voltage);

    void writePeriodic();
    void readPeriodic();

    void simulationPeriodic();

    void simulatePeriodic();
}
