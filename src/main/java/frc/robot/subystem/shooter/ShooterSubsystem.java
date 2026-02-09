package frc.robot.subystem.shooter;


import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends Shooter {
    private static ShooterSubsystem INSTANCE;

    private final ShooterIO shooter;

    private double leaderTargetSpeed;
    private double followerTargetSpeed;

    private ShooterMode shooterMode;

    private double targetTopTestRPM;
    private double targetBottomTestRPM;

    private ShooterSubsystem()
    {
        shooter = new Shooter();

//        if (Robot.isReal())
//        {
//
//        }//else{
//            shooter = new ShooterSim();
//        }

        leaderTargetSpeed = 0;
        followerTargetSpeed = 0;

        shooterMode = ShooterMode.off;

        targetTopTestRPM = 2000;
        targetBottomTestRPM = 2000;
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
                leaderTargetSpeed = 2000;
                followerTargetSpeed = 2000;
            }

            case off -> {
                leaderTargetSpeed = 0;
                followerTargetSpeed = 0;
            }
        }

        shooter.setLeaderTargetRPM(leaderTargetSpeed);
        shooter.setFollowerTargetRPM(followerTargetSpeed);
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

