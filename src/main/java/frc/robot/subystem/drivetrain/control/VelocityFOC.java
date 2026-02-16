package frc.robot.subystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VelocityFOC implements DrivetrainControlIO{

    private final double horizontalVelocity;
    private final double verticalVelocity;
    private final double omega;

    public VelocityFOC(double horiztonalVelocity, double VerticalVelocity, double omega) {
        this.horizontalVelocity = horiztonalVelocity;
        this.verticalVelocity = VerticalVelocity;
        this.omega = omega;
    }

    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        return new ChassisSpeeds(
                horizontalVelocity,
                verticalVelocity,
                omega
        );
    }
}
