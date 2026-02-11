package frc.robot.subsystem.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystem.mop.MopConstants;

public abstract class ClimberTalonFX {
    private final TalonFX climberMotor;
    private double inputVoltage;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> satorCurrentSignal;
    private final StatusSignal<Voltage> voltageSignal;

    ClimberStates states;
    public ClimberTalonFX(){
        climberMotor = new  TalonFX(ClimberConstants.CLIMBER_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = 12.0;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotor.getConfigurator().apply(config);

        climberMotor.clearStickyFaults();

        velocitySignal = climberMotor.getVelocity();
        supplyCurrentSignal = climberMotor.getSupplyCurrent();
        satorCurrentSignal = climberMotor.getSupplyCurrent();
        voltageSignal = climberMotor.getMotorVoltage();

        inputVoltage = 0;
        setVoltage(0);
    }
    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        climberMotor.set(inputVoltage);
    }

    public abstract void readPeriodic();

    public abstract void writePeriodic();

    public abstract void simulatePeriodic();
}
