package frc.robot.subystem.intake;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.Logger;


public class Intake {

    private final TalonFX pivotMotor;
    private final TalonFX spinMotor;

    private double inputVoltage;

    public Intake (){

        pivotMotor = new TalonFX(IntakeConstants.PIVOT_ID);
        spinMotor = new TalonFX(IntakeConstants.SPIN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = 12.0;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        spinMotor.getConfigurator().apply(config);
        pivotMotor.getConfigurator().apply(config);

        pivotMotor.clearStickyFaults();
        spinMotor.clearStickyFaults();

        inputVoltage = 0;
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        inputVoltage = voltage;
        pivotMotor.set(inputVoltage);
        spinMotor.set(inputVoltage);
    }


    public void readPeriodic() {

    }

    public void writePeriodic() {
        Logger.recordOutput("Intake/Pivot/Temperature", pivotMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Intake/Pivot/CurrentDraw", pivotMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Intake/Pivot/MotorVoltage", pivotMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Pivot/InputVoltage", this.inputVoltage);

        Logger.recordOutput("Intake/Spin/Temperature", spinMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Intake/Spin/CurrentDraw", spinMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Intake/Spin/MotorVoltage", spinMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Spin/InputVoltage", this.inputVoltage);
    }


}
