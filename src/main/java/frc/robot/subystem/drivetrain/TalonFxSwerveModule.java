package frc.robot.subystem.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TalonFxSwerveModule implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final PositionVoltage steerControl;

    private final CANcoder steerEncoder;
    private final SwerveModule module;

    public TalonFxSwerveModule(SwerveModule module) {
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
        steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

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
}
