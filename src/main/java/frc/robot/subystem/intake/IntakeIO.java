package frc.robot.subystem.intake;

public interface IntakeIO {

    void setVoltage(double voltage);

    void writePeriodic();
    void readPeriodic();
}
