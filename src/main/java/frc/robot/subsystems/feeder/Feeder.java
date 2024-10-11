package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public FeederIO io;
  public FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public BeambreakIO feederBeambreak;
  public BeambreakIOInputsAutoLogged feederBeambreakInputs = new BeambreakIOInputsAutoLogged();
  public BeambreakIO shooterBeambreak;
  public BeambreakIOInputsAutoLogged shooterBeambreakInputs = new BeambreakIOInputsAutoLogged();

  private SimpleMotorFeedforward ff;

  public Feeder(FeederIO io, BeambreakIO feederBeambreak, BeambreakIO shooterBeambreak) {
    this.io = io;
    this.feederBeambreak = feederBeambreak;
    this.shooterBeambreak = shooterBeambreak;

    io.setPID(FeederConstants.kPReal, 0.0, 0.0);
    ff = new SimpleMotorFeedforward(0.0, FeederConstants.kVReal);
  }

  public void periodic() {
    io.processInputs(inputs);
    feederBeambreak.processInputs(feederBeambreakInputs);
    shooterBeambreak.processInputs(shooterBeambreakInputs);
    Logger.processInputs("Feeder", inputs);
    Logger.processInputs("Feeder/FeederBeambreak", feederBeambreakInputs);
    Logger.processInputs("Feeder/ShooterBeambreak", shooterBeambreakInputs);
  }

  public Command setRPM(IntSupplier rpm) {
    return this.run(
        () -> {
          io.setRPM(rpm.getAsInt(), ff);
          inputs.targetRPM = rpm.getAsInt();
        });
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public boolean feederBeambreakObstructed() {
    return feederBeambreakInputs.isObstructed;
  }

  public boolean shooterBeambreakObstructed() {
    return shooterBeambreakInputs.isObstructed;
  }
}
