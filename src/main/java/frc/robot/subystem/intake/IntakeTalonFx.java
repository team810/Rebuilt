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
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class IntakeTalonFx implements IntakeIO {

    private final TalonFX driveMotor;
    private VoltageOut driveControl;
    private Voltage driveAppliedVoltage;

    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Temperature> driveTemperatureSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;
    private final StatusSignal<Current> driveSupplyCurrentSignal;

    private final DoubleSolenoid solenoid;

    public IntakeTalonFx() {

        driveMotor = new TalonFX(IntakeConstants.DRIVE_MOTOR_ID, Robot.MECH_CANBUS);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveControl = new VoltageOut(0);
        driveControl.EnableFOC = false;
        driveControl.UpdateFreqHz = 1000;
        driveControl.UseTimesync = false;
        driveControl.LimitReverseMotion = false;
        driveControl.LimitForwardMotion = false;

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
    public void setVoltage(Voltage voltage) {
        //driveAppliedVoltage = Volts.in(voltage.Output);
        driveControl = new VoltageOut(voltage);
        driveMotor.setControl(driveControl);
    }

    @Override
    public DoubleSolenoid.Value getPivotState(){
        return solenoid.get();
    };

}