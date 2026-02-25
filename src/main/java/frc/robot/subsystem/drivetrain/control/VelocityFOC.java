package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VelocityFOC implements DrivetrainControlIO {

    private final double horizontalVelocity;
    private final double verticalVelocity;
    private final double omega;

    public VelocityFOC(double horizontalVelocity, double verticalVelocity, double omega) {
        this.horizontalVelocity = horizontalVelocity;
        this.verticalVelocity = verticalVelocity;
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
