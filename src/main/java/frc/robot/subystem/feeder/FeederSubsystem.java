package frc.robot.subystem.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RPM;

public class FeederSubsystem extends FeederTalonFX {
    private static FeederSubsystem INSTANCE;

    private final FeederIO feeder;

    private AngularVelocity feederTargetSpeed;

    private FeederStates feederState;

    private FeederSubsystem() {
        feeder = new FeederTalonFX();
    }

    @Override
    public void readPeriodic() {
        feeder.readPeriodic();
    }

    @Override
    public void writePeriodic() {
        Logger.recordOutput("Feeder/Motor/TargetSpeed", feederTargetSpeed);
        Logger.recordOutput("Feeder/Mode/FeederMode", feederState);

        feeder.writePeriodic();
    }

    public void setShooterMode(FeederStates feederState) {
        this.feederState = feederState;

        switch (feederState)
        {
            case ON -> {
                feederTargetSpeed = AngularVelocity.ofBaseUnits(3000, RPM.getBaseUnit());
            }

            case OFF -> {
                feederTargetSpeed = AngularVelocity.ofBaseUnits(0, RPM.getBaseUnit());
            }
        }
    }


    public static FeederSubsystem getInstance()
    {
        if (INSTANCE == null)
        {
            INSTANCE = new FeederSubsystem();
        }
        return INSTANCE;
    }

}
