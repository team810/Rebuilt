package frc.robot.subsystem.intake;

public interface IntakeIO {

    void setVoltage(double voltage);

    void writePeriodic();
    void readPeriodic();
}
