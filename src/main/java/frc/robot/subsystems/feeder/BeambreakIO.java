package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {
    @AutoLog
    public static class BeambreakIOInputs {
        public boolean isObstructed = false;
    }

    public default void processInputs(BeambreakIOInputs inputs) {
    }

    public default void overrideObstructed(boolean isObstructed) {
    }
}
