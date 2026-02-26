package frc.robot.subsystem.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ClimberTalonFx implements ClimberIO{

    private final TalonFX motor;
    private final PositionVoltage control;

    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Temperature> driveTemperatureSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;
    private final StatusSignal<Current> driveSupplyCurrentSignal;

    private final DoubleSolenoid solenoid;

    public ClimberTalonFx() {

        motor = new TalonFX(ClimberConstants.MOTOR_ID, Robot.MECH_CANBUS);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Slot0.kP = 5;
        driveMotorConfig.Slot0.kI = 0;
        driveMotorConfig.Slot0.kD = 0;
        driveMotorConfig.Slot0.kG = 0;
        motor.getConfigurator().apply(driveMotorConfig);

        control = new PositionVoltage(0); // Angle
        control.UseTimesync = true;
        control.Slot = 0;
        control.EnableFOC = true;
        control.UpdateFreqHz = 1000;

        driveVoltageSignal = motor.getMotorVoltage();
        driveTemperatureSignal = motor.getDeviceTemp();
        driveAppliedCurrentSignal = motor.getStatorCurrent();
        driveSupplyCurrentSignal = motor.getSupplyCurrent();

        StatusSignal.setUpdateFrequencyForAll(
            50,
            driveVoltageSignal,
            driveTemperatureSignal,
            driveAppliedCurrentSignal,
            driveSupplyCurrentSignal
        );

        solenoid = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                ClimberConstants.SOLENOID_FWD_CHANNEL,
                ClimberConstants.SOLENOID_REV_CHANNEL
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

        Logger.recordOutput("Climber/Motor/Voltage", driveVoltageSignal.getValue());
        Logger.recordOutput("Climber/Motor/Current", driveAppliedCurrentSignal.getValue());
        Logger.recordOutput("Climber/Motor/SupplyCurrent", driveSupplyCurrentSignal.getValue());
        Logger.recordOutput("Climber/Motor/Temperature", driveTemperatureSignal.getValue().in(Celsius));

    }

    @Override
    public void setPivotState(DoubleSolenoid.Value value) {
        solenoid.set(value);
    }

    @Override
    public void setHeight(Distance targetHeight) {
        double height = targetHeight.in(Inches);
        control.Position = (height / (2 * Math.PI * 2)) * 75;
        motor.setControl(control);
    }

    @Override
    public DoubleSolenoid.Value getPivotState(){
        return solenoid.get();
    }

}
