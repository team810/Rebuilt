package frc.robot.subsystem.mop;

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


public class MopTalonFx implements MopIO {
    private final TalonFX mopMotor;
    private VoltageOut mopControl;
    private Voltage mopAppliedVoltage;

    private final StatusSignal<Voltage> mopVoltageSignal;
    private final StatusSignal<Temperature> mopTemperatureSignal;
    private final StatusSignal<Current> mopAppliedCurrentSignal;
    private final StatusSignal<Current> mopSupplyCurrentSignal;

    public MopTalonFx() {
        mopMotor = new TalonFX(MopConstants.MOP_MOTOR_ID, Robot.MECH_CANBUS);

        TalonFXConfiguration mopConfig = new TalonFXConfiguration();
        mopConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mopConfig.CurrentLimits.SupplyCurrentLimit = 40;
        mopConfig.CurrentLimits.StatorCurrentLimit = 80;
        mopConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        mopConfig.Voltage.PeakForwardVoltage = 12;
        mopConfig.Voltage.PeakReverseVoltage = -12;
        mopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mopMotor.getConfigurator().apply(mopConfig);

        mopControl = new VoltageOut(0);
        mopControl.EnableFOC = true;
        mopControl.UpdateFreqHz = 1000;
        mopControl.UseTimesync = false;
        mopControl.LimitForwardMotion = false;
        mopControl.LimitReverseMotion = false;

        mopAppliedVoltage = Volts.of(0);

        mopVoltageSignal = mopMotor.getMotorVoltage();
        mopTemperatureSignal = mopMotor.getDeviceTemp();
        mopAppliedCurrentSignal = mopMotor.getStatorCurrent();
        mopSupplyCurrentSignal = mopMotor.getSupplyCurrent();

        StatusSignal.setUpdateFrequencyForAll(50,
            mopVoltageSignal,
            mopTemperatureSignal,
            mopAppliedCurrentSignal,
            mopSupplyCurrentSignal
        );

    }
    @Override
    public void readPeriodic(){
        StatusSignal.refreshAll(
            mopVoltageSignal,
            mopTemperatureSignal,
            mopAppliedCurrentSignal,
            mopSupplyCurrentSignal
        );

        Logger.recordOutput("Mop/Motor/Voltage", mopVoltageSignal.getValue());
        Logger.recordOutput("Mop/Motor/Current" , mopAppliedCurrentSignal.getValue());
        Logger.recordOutput("Mop/Motor/SupplyCurrent" , mopSupplyCurrentSignal.getValue());
        Logger.recordOutput("Mop/Motor/Temperature" , mopTemperatureSignal.getValue());
        Logger.recordOutput("Mop/Motor/AppliedVoltage" , mopAppliedVoltage);

    }

    @Override
    public void setVoltage(Voltage voltage){
        mopControl = new VoltageOut(voltage);
        mopMotor.setControl(mopControl);
    }
}