package frc.robot.subsystem.drivetrain.control;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;

public class TrajectoryControl implements DrivetrainControlIO {
    private final PIDController horizontalController;
    private final PIDController verticalController;
    private final PIDController thetaController;

    private final TrajectorySample<SwerveSample> sample;

    public TrajectoryControl(TrajectorySample<SwerveSample> sample) {
        this.sample = sample;

        horizontalController = new PIDController(
                4,
                0,
                0
        );
        verticalController = new PIDController(
                4,
                0,
                0
        );
        thetaController = new PIDController(
                DrivetrainConstants.THETA_KP,
                DrivetrainConstants.THETA_KI,
                DrivetrainConstants.THETA_KD
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        horizontalController.setTolerance(.05);
        verticalController.setTolerance(.05);
        thetaController.setTolerance(.05);
    }

    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        ChassisSpeeds speeds = new ChassisSpeeds(
            horizontalController.calculate(currentPose.getX(), sample.getPose().getX()) + sample.getChassisSpeeds().vxMetersPerSecond,
            verticalController.calculate(currentPose.getY(), sample.getPose().getY()) + sample.getChassisSpeeds().vyMetersPerSecond,
            thetaController.calculate(currentPose.getRotation().getRadians(), sample.getPose().getRotation().getRadians()) + sample.getChassisSpeeds().omegaRadiansPerSecond
        );
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation());

        return speeds;
    }

    @Override
    public boolean atSetpoint() {
        Pose2d targetPose = sample.getPose();
        Pose2d currentPose = Drivetrain.getInstance().getPose();
        return
            MathUtil.isNear(targetPose.getX() - currentPose.getX(), 0, .1) &&
            MathUtil.isNear(targetPose.getY() - currentPose.getY(), 0, .1) &&
            MathUtil.isNear(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(), 0, .1);
    }

    @Override
    public Pose2d getTargetPose() {
        return sample.getPose();
    }
}
