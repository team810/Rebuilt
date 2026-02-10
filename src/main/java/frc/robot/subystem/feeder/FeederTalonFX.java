package frc.robot.subystem.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

public class FeederTalonFX implements FeederIO{
    private final TalonFX feederMotor;
    private final TalonFXConfiguration config;

    private final StatusSignal<AngularVelocity> feederVelocitySignal;
    private final StatusSignal<Current> feederSupplySignal;
    private final StatusSignal<Current> feederStatorSignal;
    private final StatusSignal<Voltage> feederVoltageSignal;
    private final StatusSignal<Temperature> feederTemperature;

    private final VelocityVoltage control;

    private AngularVelocity feederTargetRPM;

    public FeederTalonFX() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, FeederConstants.CANBUS);

        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.Audio.BeepOnConfig = true;
        config.Audio.BeepOnBoot = true;

        config.Slot0.kV = .1;
        config.Slot0.kA = .001;
        config.Slot0.kP = .75;
        config.Slot0.kI = 0;

        feederMotor.getConfigurator().apply(config);

        feederVelocitySignal = feederMotor.getVelocity();
        feederSupplySignal = feederMotor.getSupplyCurrent();
        feederStatorSignal = feederMotor.getSupplyCurrent();
        feederVoltageSignal = feederMotor.getMotorVoltage();
        feederTemperature = feederMotor.getDeviceTemp();

        control = new VelocityVoltage(0);
        control.Slot = 0;
        control.EnableFOC = false;
        control.IgnoreHardwareLimits = false;
        control.LimitForwardMotion = false;
        control.LimitReverseMotion = false;
        control.UpdateFreqHz = 1000;

        feederMotor.setControl(control);
    }
    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                feederTemperature,
                feederSupplySignal,
                feederStatorSignal,
                feederVoltageSignal,
                feederVelocitySignal
        );
        Logger.recordOutput("Shooter/Leader/TargetVelocity", feederTargetRPM);
        Logger.recordOutput("Shooter/Leader/Temperature", feederTemperature.getValue());
        Logger.recordOutput("Shooter/Leader/SupplyCurrent", feederSupplySignal.getValue());
        Logger.recordOutput("Shooter/Leader/StatorCurrent", feederStatorSignal.getValue());
        Logger.recordOutput("Shooter/Leader/VoltageSignal", feederVoltageSignal.getValue());
        Logger.recordOutput("Shooter/Leader/VelocitySignal", feederVelocitySignal.getValue());
    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public AngularVelocity calculateFeederRPM(Distance distance) {
        return AngularVelocity.ofBaseUnits(1510 + (217 * distance.in(Inches)) - (3.81 * distance.in(Inches) * distance.in(Inches)),RPM.getBaseUnit());
    }
}
