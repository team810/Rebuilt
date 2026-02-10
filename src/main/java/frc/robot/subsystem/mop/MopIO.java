package frc.robot.subsystem.mop;

public interface MopIO {
    void writePeriodic();
    void readPeriodic();
    public default void simulationPeriodic() {return;};

}
