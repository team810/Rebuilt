package frc.robot.subsystem.feeder;


import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class FeederSubsystem {
    private static FeederSubsystem INSTANCE = new FeederSubsystem();

    private FeederStates state;

    private final HashMap<FeederStates, Voltage> feederVoltageMap = new HashMap<>();

    private final FeederIO io;
    private FeederSubsystem() {
        state = FeederStates.OFF;

        feederVoltageMap.put(FeederStates.OFF,Voltage.ofBaseUnits(0, Units.Volts));
        feederVoltageMap.put(FeederStates.REVERSE,Voltage.ofBaseUnits(FeederConstants.REVERSE, Units.Volts));
        feederVoltageMap.put(FeederStates.FEED ,Voltage.ofBaseUnits(FeederConstants.FEED_VOLTAGE, Units.Volts));// This constant

        io = new FeederTalonFX();

        io.setVoltage(Voltage.ofBaseUnits(0, Units.Volts));
    }

    public void readPeriodic(){
        Logger.recordOutput("Feeder/State",state);
        io.writePeriodic();
    }
    public void setState(FeederStates state){
        this.state = state;
        io.setVoltage(feederVoltageMap.get(state));
    }
    public static FeederSubsystem getInstance(){
        return INSTANCE;
    }



}
