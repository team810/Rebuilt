package frc.robot.subystem.shooter;


import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

public class ShooterSubsystem extends ShooterTalonFX {
    private static ShooterSubsystem INSTANCE;

    private final ShooterIO shooter;

    private AngularVelocity leaderTargetRPM;
    private AngularVelocity followerTargetRPM;

    private ShooterStates shooterState;

    private ShooterSubsystem()
    {
        shooter = new ShooterTalonFX();

//        if (Robot.isReal())
//        {
//
//        }//else{
//            shooter = new ShooterSim();
//        }

//        leaderTargetRPM = 0;
//        followerTargetRPM = 0;

        shooterState = ShooterStates.OFF;
    }

    @Override
    public void readPeriodic() {
        shooter.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Leader/TargetSpeedSub", leaderTargetRPM);
        Logger.recordOutput("Shooter/Follower/TargetSpeedSub", followerTargetRPM);
        Logger.recordOutput("Shooter/Mode/ShooterMode", shooterState);

        shooter.writePeriodic();
    }

    public void setShooterMode(ShooterStates shooterState) {
        this.shooterState = shooterState;

        switch (shooterState)
        {
            case ON -> {
                leaderTargetRPM = calculateLeaderRPM(Inches.of(6/*odometry distance double */));
                followerTargetRPM = leaderTargetRPM;
            }

            case OFF -> {
                leaderTargetRPM = AngularVelocity.ofBaseUnits(0, RPM.getBaseUnit());
                followerTargetRPM = AngularVelocity.ofBaseUnits(0,RPM.getBaseUnit());
            }
        }
        shooter.setLeaderRPM(leaderTargetRPM);
        shooter.setFollowerRPM(followerTargetRPM);
    }


    public static ShooterSubsystem getInstance()
    {
        if (INSTANCE == null)
        {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }
}

