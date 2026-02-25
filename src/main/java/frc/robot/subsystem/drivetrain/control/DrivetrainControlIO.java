package frc.robot.subsystem.drivetrain.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DrivetrainControlIO {
    public ChassisSpeeds getSpeeds(Pose2d currentPose);
}
