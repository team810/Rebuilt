package frc.robot.subystem.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;

public class TalonFXSwerveModule implements SwerveModuleIO{
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final PositionVoltage steerControl;

    private final CANcoder steerEncoder;
    private final SwerveModule module;

    public TalonFXSwerveModule(SwerveModule module) {
        this.module = module;

        driveMotor = new TalonFX(DrivetrainConstants.getDriveID(module), DrivetrainConstants.DRIVETRAIN_CANBUS);
        steerMotor = new TalonFX(DrivetrainConstants.getSteerID(module), DrivetrainConstants.DRIVETRAIN_CANBUS);

        steerEncoder = new CANcoder(DrivetrainConstants.getEncoderID(module), DrivetrainConstants.DRIVETRAIN_CANBUS);

        steerControl = new PositionVoltage(0);
        steerControl.EnableFOC = true;
        steerControl.Slot = 0;
        steerControl.UseTimesync = true;
        steerControl.UpdateFreqHz = 1000;

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Feedback.FeedbackRemoteSensorID = steerMotor.getDeviceID();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
    @Override
    public void setTargetState(SwerveModuleState state) {

    }

    @Override
    public SwerveModuleState getCurrentState() {
        return null;
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {

    }

    @Override
    public void moduleSim() {

    }

    @Override
    public Distance getPosition() {
        return null;
    }

    @Override
    public LinearVelocity getVelocity() {
        return null;
    }

    @Override
    public LinearAcceleration getAcceleration() {
        return null;
    }

    @Override
    public Voltage getDriveAppliedVoltage() {
        return null;
    }

    @Override
    public Angle getTheta() {
        return null;
    }

    @Override
    public AngularVelocity getOmega() {
        return null;
    }

    @Override
    public Force getForce() {
        return null;
    }

    @Override
    public Torque getTorque() {
        return null;
    }

    @Override
    public Voltage getSteerAppliedVoltage() {
        return null;
    }
}
