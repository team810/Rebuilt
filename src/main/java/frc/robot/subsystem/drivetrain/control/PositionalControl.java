package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;

public class PositionalControl implements DrivetrainControlIO{
    private final PIDController horizontalController;
    private final PIDController verticalController;
    private final PIDController thetaController;

    private final Pose2d targetPose;

    public PositionalControl(Pose2d targetPose) {
        this.targetPose = targetPose;

        horizontalController = new PIDController(
            DrivetrainConstants.LINEAR_KP,
            DrivetrainConstants.LINEAR_KI,
            DrivetrainConstants.LINEAR_KD
        );
        verticalController = new PIDController(
            DrivetrainConstants.LINEAR_KP,
            DrivetrainConstants.LINEAR_KI,
            DrivetrainConstants.LINEAR_KD
        );
        thetaController = new PIDController(
            DrivetrainConstants.THETA_KP,
            DrivetrainConstants.THETA_KI,
            DrivetrainConstants.THETA_KD
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        ChassisSpeeds speeds = new ChassisSpeeds(
            horizontalController.calculate(currentPose.getX(), targetPose.getX()),
            verticalController.calculate(currentPose.getY(), targetPose.getY()),
            thetaController.calculate(currentPose.getRotation().getRadians() + targetPose.getRotation().getRadians())
        );
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation());
        return speeds;
    }
}
