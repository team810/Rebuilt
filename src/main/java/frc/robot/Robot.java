// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystem.mop.Mop;
import frc.robot.subsystem.mop.MopStates;
import frc.robot.subsystem.mop.MopSubsystem;
import frc.robot.subsystem.mop.MopTalonFX;
import org.littletonrobotics.junction.LoggedRobot;


public class Robot extends LoggedRobot {
    MopTalonFX mop;
    XboxController xboxController;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot(){
      mop = new MopTalonFX();
      xboxController = new XboxController(0);
      CommandScheduler.getInstance().setPeriod(0.015);
      Trigger mopMove =  new Trigger(()-> xboxController.getAButton());
      mopMove.whileTrue(
              new StartEndCommand(
                      ()->mop.setMopStates(MopStates.fwd),
                      ()->mop.setMopStates(MopStates.off),
                      (Subsystem) this.mop
              )
      );
  }

  @Override
  public void robotInit() {
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
  public void simulationPeriodic() {
      MopSubsystem.getInstance().simulationPeriodic();
  }
}
