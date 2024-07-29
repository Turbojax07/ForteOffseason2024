package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakIOReal implements BeambreakIO {
    private final DigitalInput beambreak;

    public BeambreakIOReal(int dioChannel) {
        this.beambreak = new DigitalInput(dioChannel);
    }

    @Override
    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isObstructed = !beambreak.get();
    }
}
