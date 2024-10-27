package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public Command setDutyCycle(double dutyCycle) {
    return run(
        () -> {
          io.setOpenLoopDutyCycle(dutyCycle);
          inputs.climberTargetSpeed = dutyCycle;
        });
  }

  // public Command runCurrentHoming() {
  //   return Commands.runOnce(() -> io.setVoltage(-1.0))
  //       .until(() -> inputs.climberCurrentAmps > 40.0)
  //       .finallyDo(() -> io.resetEncoder(0.0));
  // }

  public double getExtensionMeters() {
    return inputs.climberPositionMeters;
  }

  public double getTargetMeters() {
    return inputs.climberTargetMeters;
  }
}
