package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.Superstructure;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.control.PositionalControl;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;

import java.util.ArrayList;
import java.util.Collection;

public class DriveToCommand extends Command {
    private Pose2d target;
    private PositionalControl control;

    public DriveToCommand(Pose2d target) {
        Collection<Pose2d> poseList = new ArrayList<>();
        poseList.add(new Pose2d(5.770099639892578, 7.360217571258545, new Rotation2d(0)));
        poseList.add(new Pose2d(5.882756233215332,0.5632885694503784, new Rotation2d(0)));
        poseList.add(new Pose2d(3.509453058242798,0.5126494765281677, new Rotation2d(Math.PI)));
        poseList.add(new Pose2d(3.3292129039764404,7.322665214538574, new Rotation2d(Math.PI)));
        this.target = Drivetrain.getInstance().getPose().nearest(poseList);
        control = new PositionalControl(target);
    }

    @Override
    public void initialize() {
        Collection<Pose2d> poseList = new ArrayList<>();
        poseList.add(new Pose2d(5.770099639892578, 7.360217571258545, new Rotation2d(0)));
        poseList.add(new Pose2d(5.882756233215332,0.5632885694503784, new Rotation2d(0)));
        poseList.add(new Pose2d(3.509453058242798,0.5126494765281677, new Rotation2d(Math.PI)));
        poseList.add(new Pose2d(3.3292129039764404,7.322665214538574, new Rotation2d(Math.PI)));
        this.target = Drivetrain.getInstance().getPose().nearest(poseList);
        control = new PositionalControl(target);
        Superstructure.getInstance().setRobotState(RobotStates.Auto);
        Drivetrain.getInstance().setControl(control);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Superstructure.getInstance().setRobotState(RobotStates.Default);
        Drivetrain.getInstance().setControl(new VelocityFOC(0,0,0));
    }

    @Override
    public boolean isFinished() {
        return control.atSetpoint();
    }
}
