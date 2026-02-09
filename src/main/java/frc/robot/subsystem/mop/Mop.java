package frc.robot.subsystem.mop;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Mop implements Subsystem {
    private final TalonFX mopMotor;
    private double inputVoltage;

    MopStates states;
    public Mop(){
        mopMotor = new  TalonFX(MopConstants.MOP_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = 12.0;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mopMotor.getConfigurator().apply(config);

        mopMotor.clearStickyFaults();

        inputVoltage = 0;
        setVoltage(0);
    }
    public void setMopStates(MopStates states){
        switch (states){
            case fwd -> {
                mopMotor.set(1);
            }
            case off -> {
                mopMotor.set(0);
            }
        }
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        mopMotor.set(inputVoltage);
    }
    public void readPeriodic() {

    }

    public void writePeriodic() {
        Logger.recordOutput("Mop/Motor/Temperature", mopMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Mop/Motor/CurrentDraw", mopMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Mop/Motor/MotorVoltage", mopMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Mop/Motor/InputVoltage", this.inputVoltage);

    }

}
