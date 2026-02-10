package frc.robot.subystem.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

public class ShooterTalonFX implements ShooterIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final TalonFXConfiguration config;

    private final StatusSignal<AngularVelocity> leaderVelocitySignal;
    private final StatusSignal<Current> leaderSupplySignal;
    private final StatusSignal<Current> leaderStatorSignal;
    private final StatusSignal<Voltage> leaderVoltageSignal;
    private final StatusSignal<Temperature> leaderTemperature;


    private final StatusSignal<AngularVelocity> followerVelocitySignal;
    private final StatusSignal<Current> followerSupplySignal;
    private final StatusSignal<Current> followerStatorSignal;
    private final StatusSignal<Voltage> followerVoltageSignal;
    private final StatusSignal<Temperature> followerTemperature;


    private final VelocityVoltage control;
    private final Follower followerController;

    private AngularVelocity leaderTargetRPM;

    public ShooterTalonFX() {
        leaderMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_ID, ShooterConstants.CANBUS);
        followerMotor = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_ID, ShooterConstants.CANBUS);
        followerController = new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed);

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

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        leaderVelocitySignal = leaderMotor.getVelocity();
        leaderSupplySignal = leaderMotor.getSupplyCurrent();
        leaderStatorSignal = leaderMotor.getSupplyCurrent();
        leaderVoltageSignal = leaderMotor.getMotorVoltage();
        leaderTemperature = leaderMotor.getDeviceTemp();

        followerVelocitySignal = followerMotor.getVelocity();
        followerSupplySignal = followerMotor.getSupplyCurrent();
        followerStatorSignal = followerMotor.getStatorCurrent();
        followerVoltageSignal = followerMotor.getMotorVoltage();
        followerTemperature = followerMotor.getDeviceTemp();

        followerMotor.setControl(followerController);

        control = new VelocityVoltage(0);
        control.Slot = 0;
        control.EnableFOC = false;
        control.IgnoreHardwareLimits = false;
        control.LimitForwardMotion = false;
        control.LimitReverseMotion = false;
        control.UpdateFreqHz = 1000;

        leaderMotor.setControl(control);
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                leaderTemperature,
                leaderSupplySignal,
                leaderStatorSignal,
                leaderVoltageSignal,
                leaderVelocitySignal,

                followerTemperature,
                followerSupplySignal,
                followerVelocitySignal,
                followerVoltageSignal
        );

    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Leader/TargetVelocity", leaderTargetRPM);
        Logger.recordOutput("Shooter/Leader/Temperature", leaderTemperature.getValue());
        Logger.recordOutput("Shooter/Leader/SupplyCurrent", leaderSupplySignal.getValue());
        Logger.recordOutput("Shooter/Leader/StatorCurrent", leaderStatorSignal.getValue());
        Logger.recordOutput("Shooter/Leader/VoltageSignal", leaderVoltageSignal.getValue());
        Logger.recordOutput("Shooter/Leader/VelocitySignal", leaderVelocitySignal.getValue());

        Logger.recordOutput("Shooter/Follower/Temperature", followerTemperature.getValue());
        Logger.recordOutput("Shooter/Follower/SupplyCurrent", followerSupplySignal.getValue());
        Logger.recordOutput("Shooter/Follower/StatorCurrent", followerStatorSignal.getValue());
        Logger.recordOutput("Shooter/Follower/VoltageSignal", followerVoltageSignal.getValue());
        Logger.recordOutput("Shooter/Follower/VelocitySignal", followerVelocitySignal.getValue());
    }

    @Override
    public AngularVelocity calculateLeaderRPM(Distance distance) {
         return AngularVelocity.ofBaseUnits(1510 + (217 * distance.in(Inches)) - (3.81 * distance.in(Inches) * distance.in(Inches)),RPM.getBaseUnit());
    }

}
