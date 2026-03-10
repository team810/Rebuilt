package frc.robot.command;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.control.TrajectoryControl;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;

public class PathFollower extends Command {
    private final Trajectory<SwerveSample> trajectory;
    private final Timer timer;

    private TrajectoryControl control;

    public PathFollower(Trajectory<SwerveSample> trajectory) {
        timer = new Timer();
        this.trajectory = trajectory;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        control = new TrajectoryControl(trajectory.sampleAt(timer.get(),false).get());
        Drivetrain.getInstance().setControl(control);
    }


    @Override
    public void end(boolean interrupted) {
        Drivetrain.getInstance().setControl(new VelocityFOC(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return
            timer.hasElapsed(trajectory.getTotalTime()) &&
            control.atSetpoint();
    }
}
