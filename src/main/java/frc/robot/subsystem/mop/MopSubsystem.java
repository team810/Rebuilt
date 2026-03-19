package frc.robot.subsystem.mop;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class MopSubsystem {
    private static MopSubsystem INSTANCE = new MopSubsystem();

    private MopStates state;

    private final HashMap<MopStates, Voltage> mopVoltageMap = new HashMap<>();

    private final MopIO io;

    private MopSubsystem() {
        state = MopStates.OFF;

        mopVoltageMap.put(MopStates.OFF,Voltage.ofBaseUnits(0, Units.Volts));
        mopVoltageMap.put(MopStates.REVERSE,Voltage.ofBaseUnits(MopConstants.REVERSE, Units.Volts));
        mopVoltageMap.put(MopStates.FEED ,Voltage.ofBaseUnits(MopConstants.FEED_VOLTAGE, Units.Volts));// This constant

        io = new MopTalonFx();

        io.setVoltage(Voltage.ofBaseUnits(0, Units.Volts));

    }

    public void readPeriodic(){
        Logger.recordOutput("Mop/State",state);
        io.readPeriodic();
        io.writePeriodic();
    }
    public void setState(MopStates state){
        this.state = state;
        if(!io.getIsStuck())
            io.setVoltage(mopVoltageMap.get(state));
        else{
            double time = Logger.getTimestamp();
            double output = Math.cos(time/(1000000 * Math.PI * .5));
            if(output >= 1){
                output = 4;
            }else {
                output = -4;
            }
            io.setVoltage(Voltage.ofBaseUnits(output, Units.Volts));

        }
    }
    public static MopSubsystem getInstance(){
        return INSTANCE;
    }

}
