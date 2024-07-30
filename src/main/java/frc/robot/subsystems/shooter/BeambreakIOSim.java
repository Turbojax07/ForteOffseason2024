package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class BeambreakIOSim implements BeambreakIO {
    public final DIOSim beambreakSim;

    public BeambreakIOSim(int dioChannel) {
        beambreakSim = new DIOSim(dioChannel);
    }

    @Override
    public void processInputs(BeambreakIOInputs inputs) {
        inputs.isObstructed = !beambreakSim.getValue();
    }

    @Override
    public void overrideObstructed(boolean isObstructed) {
        this.beambreakSim.setValue(!isObstructed);
    }
}
