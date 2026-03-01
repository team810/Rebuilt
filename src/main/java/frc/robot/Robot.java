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
import frc.robot.subsystem.climber.ClimberStates;
import frc.robot.subsystem.climber.ClimberSubsystem;
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
    public static final CANBus MECH_CANBUS = new CANBus("Mech");

    private final Superstructure superstructure;

    private final Trigger resetGyroTrigger;

    private final Trigger intakeTrigger;
    private final Trigger reverseIntakeTrigger;

    private final Trigger manualClimbUpTrigger;
    private final Trigger manualClimbDownTrigger;
    private final Trigger extendClimbTrigger;
    private final Trigger retractClimbTrigger;

    private final Trigger climbAlignTrigger;
    private final Trigger shooterAlignTrigger;
    private final Trigger shooterTrigger;

    public Robot() {
        Logger.addDataReceiver(new NT4Publisher());
        if (Robot.isReal()){
            Logger.addDataReceiver(new WPILOGWriter());
        }
        Logger.start();

        IO.Init();
        superstructure = new Superstructure();
        superstructure.setAlliance(DriverStation.Alliance.Blue);
        superstructure.setRobotState(RobotStates.Default);

        CommandScheduler.getInstance().setPeriod(PERIOD);

        resetGyroTrigger = new Trigger(IO.getButton(Controls.resetGyro));

        intakeTrigger = new Trigger(IO.getButton(Controls.intake));
        intakeTrigger.whileTrue(new StartEndCommand(
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.Deployed),
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff)
        ));

        reverseIntakeTrigger = new Trigger(IO.getButton(Controls.reverseIntake));
        reverseIntakeTrigger.whileTrue(new StartEndCommand(
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.DeployedRevs),
            () -> IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff)
        ));

        manualClimbDownTrigger = new Trigger(IO.getButton(Controls.manualClimbDown)); // Need to add code in the climber subsystem to make this work
        manualClimbUpTrigger = new Trigger(IO.getButton(Controls.manualClimbUp));

        retractClimbTrigger = new Trigger(IO.getButton(Controls.retractClimb));
        retractClimbTrigger.toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setState(ClimberStates.retract)));

        extendClimbTrigger = new Trigger(IO.getButton(Controls.extendClimb));
        extendClimbTrigger.toggleOnTrue(new InstantCommand(() -> ClimberSubsystem.getInstance().setState(ClimberStates.extend)));

        climbAlignTrigger = new Trigger(IO.getButton(Controls.climbAlign));
        climbAlignTrigger.whileTrue(
            new StartEndCommand(
                () -> superstructure.setRobotState(RobotStates.AligningClimb),
                () -> superstructure.setRobotState(RobotStates.Default)
            )
        );

        shooterAlignTrigger = new Trigger(IO.getButton(Controls.alignShooting));
        shooterAlignTrigger.whileTrue(new StartEndCommand(
            () -> superstructure.setRobotState(RobotStates.Shooting),
            () -> superstructure.setRobotState(RobotStates.Default)
        ));

        shooterTrigger = new Trigger(IO.getButton(Controls.shooting));
        shooterTrigger.whileTrue(new StartEndCommand(
            () -> {
                MopSubsystem.getInstance().setState(MopStates.FEED);
                FeederSubsystem.getInstance().setState(FeederStates.FEED);
            },
            () -> {
                MopSubsystem.getInstance().setState(MopStates.OFF);
                FeederSubsystem.getInstance().setState(FeederStates.OFF);
            }
        ));
    }

    @Override
    public void robotPeriodic() {
        superstructure.readPeriodic();
        CommandScheduler.getInstance().run();
        superstructure.writePeriodic();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
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
