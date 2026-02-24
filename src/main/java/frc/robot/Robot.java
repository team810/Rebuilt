// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import org.littletonrobotics.junction.LoggedRobot;


public class Robot extends LoggedRobot {
    public static final double PERIOD = .020; // 20 milliseconds
    public static final CANBus MECH_CANBUS = new CANBus("Mech");

    public Robot() {}

  @Override
  public void robotPeriodic() {

  }

    @Override
    public void robotPeriodic() {}



    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
