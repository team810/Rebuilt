package frc.robot.subsystem.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class ClimberSubsystem {
    private static ClimberSubsystem INSTANCE = new ClimberSubsystem();

    private ClimberStates state;

    private final HashMap<ClimberStates, Distance> driveDistanceMap = new HashMap<>();
    private final HashMap<ClimberStates, DoubleSolenoid.Value> pistonMap = new HashMap<>();

    private final ClimberIO io;

    private ClimberSubsystem() {
        state = ClimberStates.retract;

        driveDistanceMap.put(ClimberStates.retract, ClimberConstants.BOTTOM_HEIGHT);
        driveDistanceMap.put(ClimberStates.extend, ClimberConstants.TOP_HEIGHT);
        driveDistanceMap.put(ClimberStates.climb, ClimberConstants.CLIMB_HEIGHT);

        pistonMap.put(ClimberStates.retract, DoubleSolenoid.Value.kReverse);
        pistonMap.put(ClimberStates.extend, DoubleSolenoid.Value.kReverse);
        pistonMap.put(ClimberStates.climb, DoubleSolenoid.Value.kForward);

        io = new ClimberTalonFx();

        io.setHeight(ClimberConstants.BOTTOM_HEIGHT);
    }

    public void readPeriodic(){
        Logger.recordOutput("Climber/State", state);
        io.readPeriodic();
    }

    public void writePeriodic(){
        io.writePeriodic();
    }

    //DoubleSolenoid.Value value
    public ClimberStates getState(){
        return state;
    }

    public void setState(ClimberStates state){
        this.state = state;
        io.setHeight(driveDistanceMap.get(state));
        io.setPivotState(pistonMap.get(state));
    }

    public static ClimberSubsystem getInstance() {
        return INSTANCE;
    }
}
