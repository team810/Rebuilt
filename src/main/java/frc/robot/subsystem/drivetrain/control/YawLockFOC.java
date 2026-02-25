package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;

public class YawLockFOC implements DrivetrainControlIO{
    private final LinearVelocity horizontalVelocity;
    private final LinearVelocity verticalVelocity;
    private final Rotation2d angleLock;

    private final PIDController thetaController;

    public YawLockFOC(LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, Rotation2d angleLock) {

        this.horizontalVelocity = horizontalVelocity;
        this.verticalVelocity = verticalVelocity;

        this.angleLock = angleLock;

        thetaController = new PIDController(
                DrivetrainConstants.THETA_KP,
                DrivetrainConstants.THETA_KI,
                DrivetrainConstants.THETA_KD
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        AngularVelocity omega = AngularVelocity.ofBaseUnits(
                thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        angleLock.getRadians()
                ),
                Units.RadiansPerSecond
        );

        ChassisSpeeds speeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omega);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation());
        return speeds;
    }
}