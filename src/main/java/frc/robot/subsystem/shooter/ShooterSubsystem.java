package frc.robot.subsystem.shooter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends ShooterTalonFX {
    private static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private Distance distanceToTarget;
    private AngularVelocity targetVelocity;

    private ShooterState state;

    private final ShooterIO shooter;


    private final SendableChooser<ShooterState> stateChooser;
    private AngularVelocity testVelocity = AngularVelocity.ofBaseUnits(0,Units.RotationsPerSecond);

    private ShooterSubsystem() {
        stateChooser = new SendableChooser<>();

        stateChooser.setDefaultOption("OFF", ShooterState.OFF);
        stateChooser.addOption("AUTO", ShooterState.AUTO);
        stateChooser.addOption("TEST", ShooterState.TEST);
        stateChooser.onChange(this::setState);
        setState(ShooterState.OFF);

        SmartDashboard.putNumber("Test RPM", testVelocity.in(RotationsPerSecond) * 60);

        shooter = new ShooterTalonFX();
    }


    @Override
    public void readPeriodic() {
        shooter.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Distance", distanceToTarget);
        Logger.recordOutput("Shooter/Velocity", targetVelocity);


        switch(state) {
            case AUTO -> {
                shooter.setVelocity(targetVelocity);
            }
            case TEST -> {
                testVelocity = AngularVelocity.ofBaseUnits(SmartDashboard.getNumber("Test RPM",0) / 60, RotationsPerSecond);
                shooter.setVelocity(testVelocity);
            }
            case OFF -> {
                shooter.setVelocity(AngularVelocity.ofBaseUnits(0, Units.RotationsPerSecond));
            }
        }

        shooter.writePeriodic();
    }

    public void setTarget(Pose2d currentPose, Pose2d targetPose) {
        distanceToTarget = Distance.ofBaseUnits(
            Math.sqrt((Math.pow(currentPose.getX(), 2) - Math.pow(targetPose.getX(), 2)) + (Math.pow(currentPose.getY(), 2)-Math.pow(targetPose.getY(), 2))),
            Units.Meters
        );
        targetVelocity = AngularVelocity.ofBaseUnits(1510 + (217 * distanceToTarget.in(Meters)) - (3.81 * distanceToTarget.in(Meters) * distanceToTarget.in(Meters)), RPM.getBaseUnit());
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public static ShooterSubsystem getInstance() {return INSTANCE;}
}

