package frc.robot.subystem.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class IntakeSubsystem {

    private static IntakeSubsystem instance;

    private IntakePivotStates pivotState;
    private IntakeDriveStates intakeState;

    private final HashMap<IntakeDriveStates, Voltage> driveVoltageMap = new HashMap<>();

    private Voltage currentTargetDriveVoltage;

    private final IntakeIO io;

    private IntakeSubsystem() {
        pivotState = IntakePivotStates.unextended;
        intakeState = IntakeDriveStates.off;

        driveVoltageMap.put(
                IntakeDriveStates.off,
                Voltage.ofBaseUnits(0, Units.Volts)
        );
        driveVoltageMap.put(IntakeDriveStates.fwd, IntakeConstants.INTAKE_VOLTAGE);
        driveVoltageMap.put(IntakeDriveStates.rev, IntakeConstants.REVERSE_VOLTAGE);

        currentTargetDriveVoltage = driveVoltageMap.get(intakeState);

        io = new IntakeTalonFx();
        io.setVoltage(new VoltageOut(0));
    }

    public void readPeriodic(){
        Logger.recordOutput("Intake/PivotState", pivotState);
        Logger.recordOutput("Intake/DriveState", intakeState);

        io.readPeriodic();

    }

    public void writePeriodic(){
        io.writePeriodic();
    }

    public void simulationPeriodic(){
        io.simulationPeriodic();
    }

    //DoubleSolenoid.Value value
    public IntakePivotStates getPivotState() {
        return pivotState;
    }

    public IntakeDriveStates getIntakeState() {
        return intakeState;
    }

    public void setDriveState(IntakeDriveStates intakeState) {
        this.intakeState = intakeState;
        currentTargetDriveVoltage = driveVoltageMap.get(this.intakeState);
        VoltageOut out = new VoltageOut(currentTargetDriveVoltage);
        if (intakeState == IntakeDriveStates.fwd) {
            out.OverrideBrakeDurNeutral = true;
        }else{
            out.OverrideBrakeDurNeutral = false;
        }
        io.setVoltage(out);
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }



}
