package frc.robot.subsystem.climber;


import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Volts;

public class ClimberSubsystem extends ClimberTalonFX {
    private static ClimberSubsystem instance;
    private ClimberStates motorState;
    private final HashMap<ClimberStates, Voltage> motorStateMap;
    private final ClimberIO io;
    private Voltage currentVoltageTarget;

    private ClimberSubsystem() {
        motorState = ClimberStates.off.off;
        motorStateMap = new HashMap<>();
        motorStateMap.put(ClimberStates.off.off, Volts.of(0));

        io = new ClimberIO() {
            @Override
            public void setVoltage(double voltage) {

            }

            @Override
            public void writePeriodic() {

            }

            @Override
            public void readPeriodic() {

            }

            @Override
            public void simulationPeriodic() {

            }

            @Override
            public void simulatePeriodic() {
                io.simulationPeriodic();
            }
        };
    }

    @Override
    public void readPeriodic() {
        io.readPeriodic();

        Logger.recordOutput("Mop/MopStates", motorState);
    }
    @Override
    public void writePeriodic() {
        io.writePeriodic();
    }

    @Override
    public void simulatePeriodic() {

    }

    public ClimberStates getMotorState() {
        return motorState;
    }
    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }

}
