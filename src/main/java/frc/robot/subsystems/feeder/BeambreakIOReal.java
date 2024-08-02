package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakIOReal implements BeambreakIO {
    private final DigitalInput beambreak;

    public BeambreakIOReal(int dioChannel) {
        this.beambreak = new DigitalInput(dioChannel);
    }

    @Override
    public void processInputs(BeambreakIOInputs inputs) {
        inputs.isObstructed = !beambreak.get();
    }
}
