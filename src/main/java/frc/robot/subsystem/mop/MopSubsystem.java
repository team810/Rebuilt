package frc.robot.subsystem.mop;


import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Volts;

public class MopSubsystem extends Mop{
    private static MopSubsystem instance;
    private MopStates motorState;
    private final HashMap<MopStates, Voltage> motorStateMap;
    private final MopIO io;
    private Voltage currentVoltageTarget;

    private MopSubsystem(){
        motorState = MopStates.off;
        motorStateMap = new HashMap<>();
        motorStateMap.put(MopStates.off, Volts.of(0));

        io = new MopTalonFX() {
            @Override
            public void simulatePeriodic() {
                
            }
        };
    }

    public void readPeriodic() {
        io.readPeriodic();

        Logger.recordOutput("Mop/MopStates", motorState);
    }
    public void writePeriodic() {
        io.writePeriodic();
    }

    
    public void simulatePeriodic() {
        io.simulationPeriodic();
    }

    public MopStates getMotorState() {
        return motorState;
    }
    public static MopSubsystem getInstance() {
        if (instance == null) {
            instance = new MopSubsystem();
        }
        return instance;
    }

    public void simulationPeriodic() {
    }
}
