package frc.robot.subystem.mop;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;

public class mopTalonFX implements mopIO {

    public final TalonFX mopmotor;
    private final PositionVoltage mopcontrol;
    private final SingleJointedArmSim mopsim;
    private TalonFXSimState mopsimstate;

    private Voltage targetVoltage;

    private final StatusSignal<Angle> moppositionsignal;
    private final StatusSignal<AngularVelocity> mopvelocitySignal;
    private final StatusSignal<Voltage> mopvoltageSignal;
    private final StatusSignal<Temperature> moptempatureSignal;
    private final StatusSignal<Current> mopAppliedCurrentSignal;
    private VoltageOut mopvoltagecontrol;
    private Voltage mopAppliedVoltage;


    public mopTalonFX() {
        mopmotor = new TalonFX(MopConstants.MOP_MOTOR_ID); // canbus says deprecated and Marked for removal
        TalonFXConfiguration mopmotorconfig = new TalonFXConfiguration();
        mopmotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mopmotorconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mopmotorconfig.CurrentLimits.SupplyCurrentLimit = 20;
        mopmotorconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        mopmotorconfig.CurrentLimits.StatorCurrentLimit = 80;
        mopmotorconfig.CurrentLimits.StatorCurrentLimitEnable = true;

        //CHANGE GEARBOX!!!!!!!!!!!!!
        mopmotorconfig.Feedback.SensorToMechanismRatio = 64;

        mopmotorconfig.Voltage.PeakForwardVoltage = 4;
        mopmotorconfig.Voltage.PeakReverseVoltage = -4;

        mopmotorconfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        mopmotorconfig.MotionMagic.MotionMagicAcceleration = 400;
        mopmotorconfig.MotionMagic.MotionMagicJerk = 100;
        mopmotor.getConfigurator().apply(mopmotorconfig);
        targetVoltage = MopConstants.Spin_VOLTAGE;

        mopcontrol = new PositionVoltage(10);
        moppositionsignal = mopmotor.getPosition();
        mopvelocitySignal = mopmotor.getVelocity();
        moptempatureSignal = mopmotor.getDeviceTemp();
        mopvoltageSignal = mopmotor.getMotorVoltage();
        mopAppliedCurrentSignal = mopmotor.getSupplyCurrent();


        /// ******DO SIM!!!!!!!!*****
        mopsim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                64,
                .148,
                edu.wpi.first.math.util.Units.inchesToMeters(6),
                MopConstants.MAX_VOLTAGE.in(Volts),
                MopConstants.MIN_VOLTAGE.in(Volts),
                true,
                MopConstants.STARTING_VOLTAGE.in(Volts)
        );
        mopsim.setState(MopConstants.STARTING_VOLTAGE.in(Volts), 0);
        if (Robot.isReal()) {
            mopmotor.setPosition(MopConstants.STARTING_VOLTAGE.in(Volts));
        }
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                moppositionsignal,
                mopvoltageSignal,
                mopvelocitySignal,
                moptempatureSignal,
                mopAppliedCurrentSignal
        );
        Logger.recordOutput("mopPosistion", moppositionsignal.getValue());
        Logger.recordOutput("mopVoltage", mopvoltageSignal.getValue());
        Logger.recordOutput("mopTempature", moptempatureSignal.getValue());
        Logger.recordOutput("mopVelocity", mopvelocitySignal.getValue());
        Logger.recordOutput("Appliledcurrent", mopAppliedCurrentSignal.getValue());

    }

    @Override
    public void writePeriodic() {
        mopcontrol.UpdateFreqHz = 1000;
        mopmotor.setControl(mopcontrol);

    }

    @Override
    public void simulatePeriodic() {
        mopsim.setInputVoltage(mopvoltageSignal.getValue().in(Volts));
        mopsim.update(Robot.defaultPeriodSecs);

        mopsimstate = mopmotor.getSimState();
        mopsimstate.setSupplyVoltage(12);
        mopsimstate.setRawRotorPosition((mopsim.getAngleRads() / (2 * Math.PI)) * 64);
        mopsimstate.setRotorVelocity((mopsim.getVelocityRadPerSec() / (2 * Math.PI)) * 64);
    }



    @Override
    public boolean atVoltageSetpoint() {
        return MathUtil.isNear(
                targetVoltage.in(Volts),
                moppositionsignal.getValue().in(Units.Radians),
                MopConstants.VOLTAGE_TOLERANCE.in(Volts)
        );
    }

    @Override
    public boolean empty() {
        return false;

    }

    @Override
    public void setDriveVoltage(VoltageOut voltage) {
        mopAppliedVoltage = Volts.of(voltage.Output);
        mopvoltagecontrol = voltage;
    }
    @Override
    public double getCurrentVoltage(){
        return mopvoltageSignal.getValue().in(Volts);
    }

    @Override
    public double getCurrentMop() {
        return moppositionsignal.getValue().in(Radians);
    }


    @Override
    public void setTargetVoltage(Voltage voltage) {
        targetVoltage = voltage;
        mopcontrol.Position = voltage.in(Volts);
    }
}