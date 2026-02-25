package frc.robot.subsystem.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    /**
     * Sets the target state of the swerve module
     * @param targetState target state of the swerve module
     */
    public void setTargetState(SwerveModuleState targetState);

    public SwerveModuleState getCurrentState();

    public SwerveModulePosition getCurrentPosition();

    public void readPeriodic();

    public void writePeriodic();

    public void simPeriodic();
}
