package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class VelocityFOC implements DrivetrainControlIO {
    private ChassisSpeeds speeds;

    public VelocityFOC(double horizontalVelocity, double verticalVelocity, double omega) {
        this.speeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omega);
    }

    public VelocityFOC(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    public VelocityFOC(LinearVelocity horizontalVelocity,  LinearVelocity verticalVelocity, AngularVelocity omega) {
        this.speeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omega);
    }

    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation());
        return speeds;
    }

    @Override
    public boolean atSetpoint() {
        return true;
    }
}
