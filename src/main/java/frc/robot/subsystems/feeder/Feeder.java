package frc.robot.subsystems.feeder;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
    public FeederIO io;
    public FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
    
    private SimpleMotorFeedforward ff;

    public Feeder(FeederIO io) {
        this.io = io;

        io.setPID(FeederConstants.kPReal, 0.0, 0.0);
        ff = new SimpleMotorFeedforward(0.0, FeederConstants.kVReal);
    }

    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs("Feeder", inputs);
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
    

        
  }
