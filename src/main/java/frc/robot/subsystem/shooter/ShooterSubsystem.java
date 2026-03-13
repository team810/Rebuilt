package frc.robot.subsystem.shooter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ShooterSubsystem extends ShooterTalonFX {
    private static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private Distance distanceToTarget;
    private double targetVelocity;

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

        SmartDashboard.putData("Shooter State", stateChooser);
        SmartDashboard.putNumber("Test RPS", testVelocity.in(RotationsPerSecond));

        shooter = new ShooterTalonFX();
    }


    @Override
    public void readPeriodic() {
        shooter.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Distance", distanceToTarget);
        Logger.recordOutput("Shooter/TargetVelocity", targetVelocity);


        switch(state) {
            case AUTO -> {
                shooter.setVelocity(targetVelocity);
            }
            case TEST -> {
                shooter.setVelocity(SmartDashboard.getNumber("Test RPS",0));
            }
            case OFF -> {
                shooter.setVelocity(0);
            }
        }

        shooter.writePeriodic();
    }

    public void setTarget(Pose2d currentPose, Pose2d targetPose, boolean ferry) {
        double x = currentPose.getX() - targetPose.getX();
        double y = currentPose.getY() - targetPose.getY();
        distanceToTarget = Distance.ofBaseUnits(
            Math.sqrt(x * x + y * y),
            Units.Meters
        );
        if (ferry) {
            distanceToTarget = Distance.ofBaseUnits(distanceToTarget.in(Meters) - 1.5, Meters);
        }
        targetVelocity = 29.9 + (9.125 * distanceToTarget.in(Meters)) - (.255 * distanceToTarget.in(Meters) * distanceToTarget.in(Meters));

    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public static ShooterSubsystem getInstance() {return INSTANCE;}
}

