package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.control.PositionalControl;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;

public class DriveToCommand extends Command {
    private final Pose2d target;
    private PositionalControl control;

    public DriveToCommand(Pose2d target) {
        this.target = target;
        control = new PositionalControl(target);
    }

    @Override
    public void initialize() {
        Drivetrain.getInstance().setControl(control);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Drivetrain.getInstance().setControl(new VelocityFOC(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return control.atSetpoint();
    }
}
