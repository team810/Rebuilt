package frc.robot.subsystem.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class FeederTalonFX implements FeederIO{
    private final TalonFX feederMotor;
    private VoltageOut feederControl;
    private Voltage feederAppliedVoltage;

    private final StatusSignal<Voltage> feederVoltageSignal;
    private final StatusSignal<Temperature> feederTemperatureSignal;
    private final StatusSignal<Current> feederAppliedCurrentSignal;
    private final StatusSignal<Current> feederSupplyCurrentSignal;


    public FeederTalonFX() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, Robot.MECH_CANBUS);

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
        feederConfig.CurrentLimits.StatorCurrentLimit = 80;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.Voltage.PeakForwardVoltage = 12;
        feederConfig.Voltage.PeakReverseVoltage = -12;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederMotor.getConfigurator().apply(feederConfig);

        feederControl = new VoltageOut(0);
        feederControl.EnableFOC = true;
        feederControl.UpdateFreqHz = 1000;
        feederControl.UseTimesync = false;
        feederControl.LimitForwardMotion = false;
        feederControl.LimitReverseMotion = false;

        feederAppliedVoltage = Volts.of(0);

        feederVoltageSignal = feederMotor.getMotorVoltage();
        feederTemperatureSignal = feederMotor.getDeviceTemp();
        feederAppliedCurrentSignal = feederMotor.getStatorCurrent();
        feederSupplyCurrentSignal = feederMotor.getSupplyCurrent();

        StatusSignal.setUpdateFrequencyForAll(50,
            feederVoltageSignal,
            feederTemperatureSignal,
            feederAppliedCurrentSignal,
            feederSupplyCurrentSignal
        );

    }
    @Override
    public void readPeriodic(){
        StatusSignal.refreshAll(
            feederVoltageSignal,
            feederTemperatureSignal,
            feederAppliedCurrentSignal,
            feederSupplyCurrentSignal
        );

        Logger.recordOutput("feeder/Motor/Voltage", feederVoltageSignal.getValue());
        Logger.recordOutput("feeder/Motor/Current" , feederAppliedCurrentSignal.getValue());
        Logger.recordOutput("feeder/Motor/SupplyCurrent" , feederSupplyCurrentSignal.getValue());
        Logger.recordOutput("feeder/Motor/Temperature" , feederTemperatureSignal.getValue());
        Logger.recordOutput("feeder/Motor/AppliedVoltage" , feederAppliedVoltage);

    }

    @Override
    public void setVoltage(Voltage voltage){
        feederControl = new VoltageOut(voltage);
        feederMotor.setControl(feederControl);
    }
}
