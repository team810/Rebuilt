package frc.robot.subystem.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class Shooter implements ShooterIO {
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final TalonFXConfiguration config;

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> satorCurrentSignal;
    private final StatusSignal<Voltage> voltageSignal;

    private final VelocityVoltage control;
    private final Follower followerController;

    private double leaderTargetRPM;
    private double followerTargetRPM;

    public Shooter() {
        leaderMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_ID, ShooterConstants.CANBUS);
        followerMotor = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_ID, ShooterConstants.CANBUS);
        followerController = new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed);

        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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

        velocitySignal = leaderMotor.getVelocity();
        supplyCurrentSignal = leaderMotor.getSupplyCurrent();
        satorCurrentSignal = leaderMotor.getSupplyCurrent();
        voltageSignal = leaderMotor.getMotorVoltage();

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
        Logger.recordOutput("Shooter/Leader/TargetVelocity", leaderTargetRPM);
//        Logger.recordOutput("Shooter/Leader/Velocity",);


    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void setLeaderTargetRPM(double targetRPM) {
        this.leaderTargetRPM = targetRPM;
    }

    @Override
    public void setFollowerTargetRPM(double targetRPM) {
        this.followerTargetRPM = targetRPM;
    }


}
