package frc.robot.subystem.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

public class FeederSubsystem extends FeederTalonFX {
    private static FeederSubsystem INSTANCE;

    private final FeederIO feeder;

    private AngularVelocity feederTargetSpeed;

    private FeederMode feederMode;

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
        Logger.recordOutput("Feeder/Mode/FeederMode", feederMode);

        feeder.writePeriodic();
    }

    public void setShooterMode(FeederMode feederMode) {
        this.feederMode = feederMode;

        switch (feederMode)
        {
            case on -> {
                feederTargetSpeed = calculateFeederRPM(Inches.of(6));
            }

            case off -> {
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
