// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystem.drivetrain.Drivetrain;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot {
    public static final double PERIOD = .020; // 20 milliseconds
    public static final CANBus MECH_CANBUS = new CANBus("Mech");

    private final Superstructure superstructure;

    private final Trigger resetGyroTrigger;

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
