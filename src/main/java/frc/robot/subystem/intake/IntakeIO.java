package frc.robot.subystem.intake;

import com.ctre.phoenix6.controls.VoltageOut;

public interface IntakeIO {
    public void readPeriodic();
    public void writePeriodic();

    void setIntakeVoltage(VoltageOut voltage);
}
