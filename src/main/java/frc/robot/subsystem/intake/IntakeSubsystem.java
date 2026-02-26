package frc.robot.subsystem.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class IntakeSubsystem {
    private static IntakeSubsystem instance = new IntakeSubsystem();

    private IntakeStates state;

    private final HashMap<IntakeStates, Voltage> driveVoltageMap = new HashMap<>();
    private final HashMap<IntakeStates, DoubleSolenoid.Value> pistonMap = new HashMap<>();

    private final IntakeIO io;

    private IntakeSubsystem() {
        state = IntakeStates.StoredOff;

        driveVoltageMap.put(IntakeStates.StoredOff, Voltage.ofBaseUnits(0, Units.Volts));
        driveVoltageMap.put(IntakeStates.Deployed, IntakeConstants.INTAKE_VOLTAGE);
        driveVoltageMap.put(IntakeStates.StoredFwd, IntakeConstants.INTAKE_VOLTAGE);
        driveVoltageMap.put(IntakeStates.DeployedRevs, IntakeConstants.REVERSE_VOLTAGE);
        driveVoltageMap.put(IntakeStates.StoredRevs, IntakeConstants.REVERSE_VOLTAGE);


        pistonMap.put(IntakeStates.StoredOff, DoubleSolenoid.Value.kReverse);
        pistonMap.put(IntakeStates.StoredFwd, DoubleSolenoid.Value.kReverse);
        pistonMap.put(IntakeStates.StoredRevs, DoubleSolenoid.Value.kReverse);
        pistonMap.put(IntakeStates.Deployed, DoubleSolenoid.Value.kForward);
        pistonMap.put(IntakeStates.DeployedRevs, DoubleSolenoid.Value.kForward);

        io = new IntakeTalonFx();

        io.setVoltage(Voltage.ofBaseUnits(0, Units.Volts));
    }

    public void readPeriodic(){
        Logger.recordOutput("Intake/State", state);
        io.readPeriodic();
    }

    public void writePeriodic(){
        io.writePeriodic();
    }

    //DoubleSolenoid.Value value
    public IntakeStates getState(){
        return state;
    }

    public void setState(IntakeStates state){
        this.state = state;
        io.setVoltage(driveVoltageMap.get(state));
        io.setPivotState(pistonMap.get(state));
    }

    public static IntakeSubsystem getInstance() {
        return instance;
    }
}
