package frc.robot.subystem.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class IntakeTalonFx implements IntakeIO {

    private final TalonFX driveMotor;
    private VoltageOut driveVoltageControl;
    private final Voltage driveAppliedVoltage;

    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Temperature> driveTemperatureSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;
    private final StatusSignal<Current> driveSupplyCurrentSignal;

    private final DoubleSolenoid solenoid;

    public IntakeTalonFx() {

        driveMotor = new TalonFX(IntakeConstants.DRIVE_MOTOR_ID, IntakeConstants.CANBUS);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveVoltageControl = new VoltageOut(0);
        driveVoltageControl.EnableFOC = false;
        driveVoltageControl.UpdateFreqHz = 1000;
        driveVoltageControl.UseTimesync = false;
        driveVoltageControl.LimitReverseMotion = false;
        driveVoltageControl.LimitForwardMotion = false;

        driveAppliedVoltage = Volts.of(0);

        driveVoltageSignal = driveMotor.getMotorVoltage();
        driveTemperatureSignal = driveMotor.getDeviceTemp();
        driveAppliedCurrentSignal = driveMotor.getStatorCurrent();
        driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();

        solenoid = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                IntakeConstants.SOLENOID_FWD_CHANNEL,
                IntakeConstants.SOLENOID_REV_CHANNEL
        );
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                driveVoltageSignal,
                driveTemperatureSignal,
                driveAppliedCurrentSignal,
                driveSupplyCurrentSignal
        );

        Logger.recordOutput("Intake/Drive/Voltage", driveVoltageSignal.getValue());
        Logger.recordOutput("Intake/Drive/Current", driveAppliedCurrentSignal.getValue());
        Logger.recordOutput("Intake/Drive/SupplyCurrent", driveSupplyCurrentSignal.getValue());
        Logger.recordOutput("Intake/Drive/Temperature", driveTemperatureSignal.getValue().in(Celsius));
        Logger.recordOutput("Intake/Drive/AppliedVoltage", driveAppliedVoltage);

    }

    @Override
    public void setPivotState(DoubleSolenoid.Value value) {
        solenoid.set(value);
    }

    @Override
    public void setVoltage(VoltageOut voltage) {
        //driveAppliedVoltage = Volts.in(voltage.Output);
        driveVoltageControl = voltage;
        driveMotor.setControl(driveVoltageControl);
    }

    @Override
    public DoubleSolenoid.Value getPivotState(){
        return solenoid.get();
    };

}