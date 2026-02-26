package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class VelocityRR implements DrivetrainControlIO{
    private ChassisSpeeds speeds;

    public VelocityRR(double horizontalVelocity, double verticalVelocity, double omega) {
        speeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omega);
    }
    public VelocityRR(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }
    public VelocityRR(LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, AngularVelocity omega) {
        this.speeds = new ChassisSpeeds(horizontalVelocity, verticalVelocity, omega);
    }
    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        return speeds;
    }
    @Override
    public boolean atSetpoint() {
        return true;
    }
}
