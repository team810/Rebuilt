package frc.robot.subsystem.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystem.drivetrain.control.DrivetrainControlIO;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;
import org.littletonrobotics.junction.Logger;

public class Drivetrain {
    private static final Drivetrain instance = new Drivetrain();

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSimState;
    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<AngularVelocity> omegaSingal;

    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    private final SwerveDrivePoseEstimator odometry;

    private DrivetrainControlIO control;

    private Drivetrain() {
        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.DRIVETRAIN_CANBUS);
        gyroSimState = new Pigeon2SimState(gyro);

        gyro.setYaw(0);
        yawSignal = gyro.getYaw();
        omegaSingal = gyro.getAngularVelocityZDevice();
        StatusSignal.setUpdateFrequencyForAll(250, yawSignal, omegaSingal);

        frontLeft = new TalonFxSwerveModule(SwerveModule.FrontLeft);
        frontRight = new TalonFxSwerveModule(SwerveModule.FrontRight);
        backLeft = new TalonFxSwerveModule(SwerveModule.BackLeft);
        backRight = new TalonFxSwerveModule(SwerveModule.BackRight);

        odometry = new SwerveDrivePoseEstimator(
                DrivetrainConstants.KINEMATICS,
                new Rotation2d(0), // gyro
                new SwerveModulePosition[] {
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition()
                },
                new Pose2d(0,0,new Rotation2d())
        );

        control = new VelocityFOC(0,0,0);
    }

    public void readPeriodic() {
        StatusSignal.refreshAll(yawSignal, omegaSingal);

        frontLeft.readPeriodic();
        frontRight.readPeriodic();
        backLeft.readPeriodic();
        backRight.readPeriodic();

        odometry.update(
            new Rotation2d(yawSignal.getValue().in(Units.Radians)),
            new SwerveModulePosition[] {
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition()
            }
        );

        // Vision Updating

        addVision(DrivetrainConstants.LIMELIGHT_SHOOTER);

        Logger.recordOutput("Pose", odometry.getEstimatedPosition());
        Logger.recordOutput(
            "Drivetrain/CurrentSwerveModuleStates",
            frontLeft.getCurrentState(),
            frontRight.getCurrentState(),
            backLeft.getCurrentState(),
            backRight.getCurrentState()
        );

    }

    private void addVision(String cam) {
        if (Robot.isReal()) {
            LimelightHelpers.SetRobotOrientation(
                cam,
                odometry.getEstimatedPosition().getRotation().getDegrees(),
                omegaSingal.getValue().in(Units.DegreesPerSecond),
                0,
                0,
                0,
                0
            );
            LimelightHelpers.PoseEstimate results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);

            if (DrivetrainConstants.USING_VISION && results != null && omegaSingal.getValue().in(Units.RadiansPerSecond) < 2 * Math.PI && results.avgTagArea > .01){

                double xyStds = Math.sqrt((.6 * (results.avgTagArea + .25))) - .15;

                if (results.tagCount >= 2){
                    xyStds *= .4;
                }
                odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 250000000));
                odometry.addVisionMeasurement(results.pose, results.timestampSeconds);
            }
        }
    }

    public void writePeriodic() {
        SwerveModuleState[] states = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(
                control.getSpeeds(getPose())
        );

        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);

        Logger.recordOutput("Drivetrain/TargetStates",states);
    }

    public void simulationPeriodic() {
        gyroSimState.addYaw((control.getSpeeds(getPose()).omegaRadiansPerSecond / (2 * Math.PI)) * 360 * Robot.PERIOD);

        frontLeft.simPeriodic();
        frontRight.simPeriodic();
        backLeft.simPeriodic();
        backRight.simPeriodic();
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
    }

    public void setControl(DrivetrainControlIO control) {
        this.control = control;
    }

    public static Drivetrain getInstance() {
        return instance;
    }
}
