package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class TalonFxSwerveModule implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFXSimState driveMotorSimState;
    private final DCMotorSim driveMotorSim;
    private final VelocityVoltage driveControl;

    private final StatusSignal<Angle> driveAngleSignal;
    private final StatusSignal<AngularVelocity> driveAngularVelocitySignal;
    private final StatusSignal<AngularAcceleration> driveAngularAccelerationSignal;
    private final StatusSignal<Temperature> driveMotorTempSignal;
    private final StatusSignal<Voltage> driveAppliedVoltageSignal;
    private final StatusSignal<Current> driveSupplyCurrentSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;

    private final TalonFX steerMotor;
    private final PositionVoltage steerControl;

    private final StatusSignal<Temperature> steerMotorTempSignal;
    private final StatusSignal<Voltage> steerAppliedVoltageSignal;
    private final StatusSignal<Current> steerSupplyCurrentSignal;
    private final StatusSignal<Current> steerAppliedCurrentSignal;

    private final CANcoder steerEncoder;
    private final CANcoderSimState steerEncoderSim;
    private final StatusSignal<Angle> steerAngleSignal;
    private final StatusSignal<AngularVelocity> steerAngularVelocitySignal;

    private SwerveModuleState targetState; // Set by drivetrain
    private SwerveModuleState currentState; // Read from encoder data

    private SwerveModulePosition modulePosition; // This is for odometry

    private final SwerveModule id;
    private final String idString;

    public TalonFxSwerveModule(SwerveModule id) {
        this.id = id;
        this.idString = id.toString();

        driveMotor = new TalonFX(DrivetrainConstants.getDriveID(id), DrivetrainConstants.DRIVETRAIN_CANBUS);
        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig());

        driveMotorSimState = new TalonFXSimState(driveMotor);
        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),.000000000001,1),DCMotor.getKrakenX60Foc(1));

        steerMotor = new TalonFX(DrivetrainConstants.getSteerID(id), DrivetrainConstants.DRIVETRAIN_CANBUS);
        steerMotor.getConfigurator().apply(DrivetrainConstants.getSteerConfig());

        steerEncoder = new CANcoder(DrivetrainConstants.getEncoderID(id), DrivetrainConstants.DRIVETRAIN_CANBUS);
        steerEncoderSim = new CANcoderSimState(steerEncoder);

        driveControl = new VelocityVoltage(0);
        driveControl.EnableFOC = true;
        driveControl.Slot = 0;
        driveControl.UseTimesync = true;
        driveControl.UpdateFreqHz = 1000;

        driveMotor.setPosition(0);

        driveAngleSignal = driveMotor.getPosition();
        driveAngularVelocitySignal = driveMotor.getVelocity();
        driveAngularAccelerationSignal = driveMotor.getAcceleration();
        driveMotorTempSignal = driveMotor.getDeviceTemp();
        driveAppliedVoltageSignal = driveMotor.getMotorVoltage();
        driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();
        driveAppliedCurrentSignal = driveMotor.getStatorCurrent();

        steerControl = new PositionVoltage(0);
        steerControl.EnableFOC = true;
        steerControl.Slot = 0;
        steerControl.UseTimesync = true;
        steerControl.UpdateFreqHz = 1000;

        steerMotorTempSignal = steerMotor.getDeviceTemp();
        steerAppliedVoltageSignal = steerMotor.getMotorVoltage();
        steerSupplyCurrentSignal = steerMotor.getSupplyCurrent();
        steerAppliedCurrentSignal = steerMotor.getStatorCurrent();

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

        steerAngleSignal = steerEncoder.getAbsolutePosition();
        steerAngularVelocitySignal = steerEncoder.getVelocity();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        targetState = new SwerveModuleState();
        currentState = new SwerveModuleState();
        modulePosition = new SwerveModulePosition();

        StatusSignal.setUpdateFrequencyForAll(
            250,
            driveAngleSignal,
            driveAngularVelocitySignal,
            driveAngularAccelerationSignal,

            steerAngleSignal,
            steerAngularVelocitySignal
        );

        StatusSignal.setUpdateFrequencyForAll(
            5,
            driveMotorTempSignal,
            driveAppliedVoltageSignal,
            driveSupplyCurrentSignal,
            driveAppliedCurrentSignal,

            steerMotorTempSignal,
            steerAppliedVoltageSignal,
            steerSupplyCurrentSignal,
            steerAppliedCurrentSignal
        );
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
            driveAngleSignal,
            driveAngularVelocitySignal,
            driveAngularAccelerationSignal,
            driveMotorTempSignal,
            driveAppliedVoltageSignal,
            driveSupplyCurrentSignal,
            driveAppliedCurrentSignal,

            steerMotorTempSignal,
            steerAppliedVoltageSignal,
            steerSupplyCurrentSignal,
            steerAppliedCurrentSignal,

            steerAngleSignal,
            steerAngularVelocitySignal
        );

        Distance distanceTraveled = Distance.ofBaseUnits(
            (driveAngleSignal.getValue().in(Units.Rotations) / 5.9) * // Gives us rotations per second of wheel
                (2 * Math.PI * .0508), // .0508 is 2'' in meters
            Units.Meters
        );

        LinearVelocity linearVelocity = LinearVelocity.ofBaseUnits(
            (driveAngularVelocitySignal.getValue().in(Units.RotationsPerSecond) / 5.9) * // Gives us rotations per second of wheel
                (2 * Math.PI * .0508), // .0508 is 2'' in meters
            Units.MetersPerSecond
        );

        currentState = new SwerveModuleState(
            linearVelocity,
            Rotation2d.fromRadians(steerAngleSignal.getValue().in(Units.Radians))
        );

        modulePosition = new SwerveModulePosition(
            distanceTraveled,
            Rotation2d.fromRadians(steerAngleSignal.getValue().in(Units.Radians))
        );


        Logger.recordOutput("Drivetrain/" + idString + "/CurrentState", currentState);
        Logger.recordOutput("Drivetrain/" + idString + "/TargetState", targetState);
        Logger.recordOutput("Drivetrain/" + idString + "/ModulePosition", modulePosition);

        Logger.recordOutput("Drivetrain/" + idString + "/Drive/Angle", driveAngleSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/LinearVelocity", linearVelocity);
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AngularVelocity", driveAngularVelocitySignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AngularAcceleration", driveAngularAccelerationSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/Temperature", driveMotorTempSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AppliedVoltage", driveAppliedVoltageSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/SupplyCurrent", driveSupplyCurrentSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Drive/AppliedCurrent", driveAppliedCurrentSignal.getValue());

        Logger.recordOutput("Drivetrain/" + idString + "/Steer/MotorTemperature", steerMotorTempSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/AppliedVoltage", steerAppliedVoltageSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/SupplyCurrent", steerSupplyCurrentSignal.getValue());
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/AppliedCurrent", steerAppliedCurrentSignal.getValue());

        Logger.recordOutput("Drivetrain/" + idString + "/Steer/Angle", steerAngleSignal.getValue().in(Units.Radians));
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/AngularVelocity", steerAngularVelocitySignal.getValue());
    }

    @Override
    public void writePeriodic() {
        targetState.optimize(currentState.angle);
        targetState.cosineScale(currentState.angle);

        AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(
            (targetState.speedMetersPerSecond/ (2 * Math.PI * .0508)) * 5.9,
            Units.RotationsPerSecond
        );

        driveControl.Velocity = targetVelocity.in(Units.RotationsPerSecond);
        driveMotor.setControl(driveControl);

        steerControl.Position = targetState.angle.getRotations();
        steerMotor.setControl(steerControl);

        Logger.recordOutput("Drivetrain/" + idString + "/Drive/TargetAngularVelocity", targetVelocity);
        Logger.recordOutput("Drivetrain/" + idString + "/Steer/TargetAngle", targetState.angle.getRadians());
    }

    @Override
    public void simPeriodic() {
        driveMotorSim.setInputVoltage(driveMotorSimState.getMotorVoltage());
        driveMotorSim.update(Robot.PERIOD);

        driveMotorSimState.setSupplyVoltage(12);

        driveMotorSimState.setRawRotorPosition(driveMotorSim.getAngularPosition());
        driveMotorSimState.setRotorVelocity(driveMotorSim.getAngularVelocity());

        // Voltage -> Angular Velocity of the wheel
        AngularVelocity simOmega = AngularVelocity.ofBaseUnits(
            ((steerAppliedVoltageSignal.getValue().in(Units.Volts) / 12) * (5800 / 60)) / ((150/7)),
            Units.RotationsPerSecond
        );
        steerEncoderSim.setSupplyVoltage(12);
        steerEncoderSim.setVelocity(simOmega);
        steerEncoderSim.addPosition(simOmega.in(Units.RotationsPerSecond) * Robot.PERIOD);
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    @Override
    public SwerveModulePosition getCurrentPosition() {
        return modulePosition;
    }

}
