package frc.robot.subystem.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    /**
     * Sets the target state of the swerve module
     * @param state target state of the swerve module
     */
    public void setTargetState(SwerveModuleState state);

    public SwerveModuleState getCurrentState();

    public void readPeriodic();

    public void writePeriodic();
}
