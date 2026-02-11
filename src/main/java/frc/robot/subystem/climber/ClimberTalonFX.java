package frc.robot.subystem.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ClimberTalonFX implements ClimberIO{
    private final TalonFX motor;
    private final VoltageOut voltageControl;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Temperature> tempatureSignal;
    private final StatusSignal<Current> appliedCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private double testVoltage;

    public ClimberTalonFX(){
        motor = new TalonFX(ClimberConstatnts.MOTOR_ID); //canbus says deprecated and marked for removal
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        motor.getConfigurator().apply(config);
        voltageControl = new VoltageOut(0);
        voltageControl.EnableFOC = true;

        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        appliedCurrentSignal = motor.getStatorCurrent();
        tempatureSignal = motor.getDeviceTemp();

        supplyVoltageSignal = motor.getSupplyVoltage();
    }
    @Override
    public void setVoltage(Voltage voltage) {
        voltageControl.Output = voltage.in(Volts);
        motor.setControl(voltageControl);
    }

    @Override
    public boolean extended() {
        return appliedCurrentSignal.getValue().in(Amps) > 95;

    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(appliedCurrentSignal, supplyCurrentSignal, tempatureSignal,voltageSignal,supplyVoltageSignal);
        Logger.recordOutput("ClimberAppliedCurrent", appliedCurrentSignal.getValue());
        Logger.recordOutput("ClimberSupplyCurrent", supplyCurrentSignal.getValue());
        Logger.recordOutput("ClimberSupplyVoltage", supplyVoltageSignal.getValue());
        Logger.recordOutput("ClimberTempature", tempatureSignal.getValue());
        Logger.recordOutput("ClimberMotorVoltage", voltageSignal.getValue());
    }
}
