package frc.robot.subystem.climber;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

import static edu.wpi.first.units.Units.*;

public class ClimberSubsystem {
        private static ClimberSubsystem instance;
        private ClimberState climberstate;

        private final HashMap<ClimberState, Voltage> climberStatemap;
        private Voltage currentVoltageTarget;

        private final ClimberIO io;

        private ClimberSubsystem(){
            climberstate = ClimberState.Off;

            climberStatemap = new HashMap<>();
            climberStatemap.put(ClimberState.Off, Volts.of(0));
            climberStatemap.put(ClimberState.Extended, ClimberConstatnts.EXTENDED_VOLTAGE);
            climberStatemap.put(ClimberState.Retracted, ClimberConstatnts.RETRACTED_VOLTAGE);

            io = new ClimberTalonFX();
        }
        public void readPeriodic(){
            io.readPeriodic();
            Logger.recordOutput("ClimberMotorState", climberstate);
        }
        public void writePeriodic(){
            io.writePeriodic();
        }
        public void simulatioPeriodic(){
            io.simulationPeriodic();
        }
        public ClimberState getMotorState(){
            return climberstate;
        }
        public boolean extended(){
            return io.extended();
        }
        public void setClimberMotorState(){
            this.climberstate = climberstate;
            this.currentVoltageTarget = climberStatemap.get(this.climberstate);
            this.io.setVoltage(currentVoltageTarget);
        }
        public static ClimberSubsystem getInstance(){
            if (instance == null){
                instance = new ClimberSubsystem();
            }
            return instance;
        }
}