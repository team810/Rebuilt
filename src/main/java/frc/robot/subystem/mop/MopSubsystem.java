package frc.robot.subystem.mop;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import java.util.HashMap;

public class MopSubsystem {
    private static MopSubsystem instance;

    private mop__states mopstates;

    private final HashMap<mop__states, Voltage> hashMap = new HashMap<>();

    private Voltage currentTargetMopVoltage;
    private final mopIO io;

    private MopSubsystem() {
        mopstates = mopstates.off;

        hashMap.put(mop__states.off, MopConstants.STORED_VOLTAGE);

        currentTargetMopVoltage = hashMap.get(mopstates);

        io = new mopTalonFX();
        io.setDriveVoltage(new VoltageOut(0));
        io.setTargetVoltage(currentTargetMopVoltage);
    }

    public void readPeriodic() {
        Logger.recordOutput("mopstate", mopstates);
        io.readPeriodic();
    }

    public void writePeriodic() {
        io.writePeriodic();
    }

    public void simulatePeriodic() {
        io.simulatePeriodic();
    }

    public boolean atVoltageSetpoint() {
        return io.atVoltageSetpoint();
    }

    public double currentVoltage() {
        return io.getCurrentVoltage();
    }

    public mop__states getMopstates() {
        return mopstates;
    }

    public void setMopstates(mop__states mopstate) {
        this.mopstates = mopstate;
        currentTargetMopVoltage = hashMap.get(this.mopstates);
        VoltageOut Out = new VoltageOut(currentTargetMopVoltage);
        if (mopstate == mop__states.spin) {
            Out.OverrideBrakeDurNeutral = true;
        } else {
            Out.OverrideBrakeDurNeutral = false;
        }
        io.setDriveVoltage(Out);
    }

    public boolean empty() {
        return io.empty();
    }

    public static MopSubsystem getInstance() {
        if (instance == null) {
            instance = new MopSubsystem();
        }
        return instance;
    }
}