package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final Mechanism2d mech2d = new Mechanism2d(ClimberConstants.width, ClimberConstants.maxHeight);
    private final MechanismRoot2d root = mech2d.getRoot("Climber", 0, 0);
    private final MechanismLigament2d hook = new MechanismLigament2d("Hook", 0, 90);

    public Climber(ClimberIO io) {
        this.io = io;

        root.append(hook);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        hook.setLength(inputs.climberPositionMeters);
        Logger.recordOutput("Climber/Mechanism2d", mech2d);
        Logger.recordOutput("Climber/Hook", hook.getLength());
    }

    public Command setExtensionCmd(DoubleSupplier meters) {
        return this.run(
            () -> {
                io.setTarget(meters.getAsDouble());
                Logger.recordOutput("Climber/Setpoint", meters.getAsDouble());
            }
        );
    }

    public Command runCurrentHoming() {
        return this.run(() -> io.setVoltage(-1.0))
        .until(() -> inputs.climberCurrentAmps > 40.0)
        .finallyDo(() -> io.resetEncoder(0.0));
    }

    public double getExtensionMeters() {
        return inputs.climberPositionMeters;
      }
}