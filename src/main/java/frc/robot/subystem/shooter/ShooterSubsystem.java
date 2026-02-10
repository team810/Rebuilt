package frc.robot.subystem.shooter;


import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

public class ShooterSubsystem extends ShooterTalonFX {
    private static ShooterSubsystem INSTANCE;

    private final ShooterIO shooter;

    private AngularVelocity leaderTargetSpeed;
    private AngularVelocity followerTargetSpeed;

    private ShooterMode shooterMode;

//    private double targetTopTestRPM;
//    private double targetBottomTestRPM;

    private ShooterSubsystem()
    {
        shooter = new ShooterTalonFX();

//        if (Robot.isReal())
//        {
//
//        }//else{
//            shooter = new ShooterSim();
//        }

//        leaderTargetSpeed = 0;
//        followerTargetSpeed = 0;

        shooterMode = ShooterMode.off;

//        targetTopTestRPM = 2000;
//        targetBottomTestRPM = 2000;
    }

    @Override
    public void readPeriodic() {
        shooter.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Shooter/Leader/TargetSpeedSub", leaderTargetSpeed);
        Logger.recordOutput("Shooter/Follower/TargetSpeedSub", followerTargetSpeed);
        Logger.recordOutput("Shooter/Mode/ShooterMode", shooterMode);

        shooter.writePeriodic();
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;

        switch (shooterMode)
        {
            case on -> {
                leaderTargetSpeed = calculateLeaderRPM(Inches.of(6));
                followerTargetSpeed = leaderTargetSpeed;
            }

            case off -> {
                leaderTargetSpeed = AngularVelocity.ofBaseUnits(0, RPM.getBaseUnit());
                followerTargetSpeed = AngularVelocity.ofBaseUnits(0,RPM.getBaseUnit());
            }
        }
        //shooter.LeaderTargetRPM(shooterMode);
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

