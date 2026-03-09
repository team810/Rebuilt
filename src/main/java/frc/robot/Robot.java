// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.feeder.FeederStates;
import frc.robot.subsystem.feeder.FeederSubsystem;
import frc.robot.subsystem.intake.IntakeStates;
import frc.robot.subsystem.intake.IntakeSubsystem;
import frc.robot.subsystem.mop.MopStates;
import frc.robot.subsystem.mop.MopSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot {
    public static final double PERIOD = .020; // 20 milliseconds
    public static final CANBus MECH_CANBUS = new CANBus("mech");

    private final Trigger resetGyroTrigger;

    private final Trigger intakeTrigger;
    private final Trigger reverseIntakeTrigger;

    private final Trigger shooterAlignTrigger;
    private final Trigger shooterTrigger;

    private final Trigger toggleIntakeTrigger;


    public Robot() {
        Logger.addDataReceiver(new NT4Publisher());
        if (Robot.isReal()){
            Logger.addDataReceiver(new WPILOGWriter());
        }
        Logger.start();

        IO.Init();
        Superstructure.getInstance().setAlliance(DriverStation.Alliance.Blue);
        Superstructure.getInstance().setRobotState(RobotStates.Default);

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().setPeriod(PERIOD);

        resetGyroTrigger = new Trigger(IO.getButton(Controls.resetGyro));
        resetGyroTrigger.onTrue(new InstantCommand(() -> Superstructure.getInstance().resetGyro()));

        toggleIntakeTrigger = new Trigger(IO.getButton(Controls.toggleIntake));
        toggleIntakeTrigger.onTrue(new InstantCommand(() -> {
            if (IntakeSubsystem.getInstance().getState() == IntakeStates.Deployed || IntakeSubsystem.getInstance().getState() == IntakeStates.DeployedRevs) {
                IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff);
            }else {
                IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
            }
        }));

        intakeTrigger = new Trigger(IO.getButton(Controls.intake));
        intakeTrigger.onTrue(new InstantCommand(
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.Deployed)
        ));

        reverseIntakeTrigger = new Trigger(IO.getButton(Controls.reverseIntake));
        reverseIntakeTrigger.whileTrue(new StartEndCommand(
            () -> {
                IntakeSubsystem.getInstance().setState(IntakeStates.DeployedRevs);
                FeederSubsystem.getInstance().setState(FeederStates.REVERSE);
                MopSubsystem.getInstance().setState(MopStates.REVERSE);
                },
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff)
        ));

        shooterAlignTrigger = new Trigger(IO.getButton(Controls.alignShooting));
        shooterAlignTrigger.whileTrue(new StartEndCommand(
            () -> Superstructure.getInstance().setRobotState(RobotStates.Shooting),
            () -> {
                Superstructure.getInstance().setRobotState(RobotStates.Default);
                MopSubsystem.getInstance().setState(MopStates.OFF);
                FeederSubsystem.getInstance().setState(FeederStates.OFF);

            }
        ));

        shooterTrigger = new Trigger(IO.getButton(Controls.shooting));
//        shooterTrigger.whileTrue(new StartEndCommand(
//            () -> {
//                MopSubsystem.getInstance().setState(MopStates.FEED);
//                FeederSubsystem.getInstance().setState(FeederStates.FEED);
//            },
//            () -> {
//                MopSubsystem.getInstance().setState(MopStates.OFF);
//                FeederSubsystem.getInstance().setState(FeederStates.OFF);
//            }
//        ));
    }

    @Override
    public void robotPeriodic() {
        Superstructure.getInstance().readPeriodic();
        CommandScheduler.getInstance().run();
        Superstructure.getInstance().writePeriodic();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(Superstructure.getInstance().getAutonomousCommand());
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        LimelightHelpers.SetThrottle("limelight-ashoote", 0);
        LimelightHelpers.SetThrottle("limelight-bshoote", 0);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        LimelightHelpers.SetThrottle("limelight-ashoote", 200);
        LimelightHelpers.SetThrottle("limelight-bshoote", 200);
    }

    @Override
    public void disabledPeriodic() {
        Superstructure.getInstance().setAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        Drivetrain.getInstance().simulationPeriodic();
    }
}
