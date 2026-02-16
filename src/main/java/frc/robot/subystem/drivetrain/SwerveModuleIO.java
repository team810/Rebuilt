package frc.robot.subystem.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;

public interface SwerveModuleIO {
    /**
     * Sets the target state of swerve module
     * @param state target state of swerve module
     */
    public void setTargetState(SwerveModuleState state);

    public SwerveModuleState getCurrentState();


    public void readPeriodic();

    /**
     * This should be called periodically after the swerve module state is set
     */
    public void writePeriodic();

    /**
     * This should be called in the drivetrain subsystem sim periodic function
     */
    void moduleSim();


    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    Distance getPosition();


    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    LinearVelocity getVelocity();

    /**
     * @return The value was last updated during the read periodic, if you need at a specific timestamp for odometry use the overload that takes in a timestamp
     */
    LinearAcceleration getAcceleration();


    /**
     * @return returns the voltage applied to the drive motor.
     */
    Voltage getDriveAppliedVoltage();

    /**
     * @return This is the current angle the wheel is facing in radians wrapped from -PI to PI
     */
    Angle getTheta();

    /**
     * @return This is the current angular velocity of the wheel
     */
    AngularVelocity getOmega();

    /**
     * @return the horizontal force created by the module
     */
    Force getForce();

    /**
     * @return Returns of the spinning wheel
     */
    Torque getTorque();

    /**
     * @return returns the voltage applied to the steer motor.
     */
    Voltage getSteerAppliedVoltage();

}
