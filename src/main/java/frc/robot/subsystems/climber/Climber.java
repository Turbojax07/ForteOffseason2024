package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP");
    private LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI");
    private LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD");

    private LoggedTunableNumber kS = new LoggedTunableNumber("Climber/kS");
    private LoggedTunableNumber kG = new LoggedTunableNumber("Climber/kG");
    private LoggedTunableNumber kV = new LoggedTunableNumber("Climber/kV");
    private LoggedTunableNumber kA = new LoggedTunableNumber("Climber/kA");
    private LoggedTunableNumber kFF = new LoggedTunableNumber("Climber/kFF");

    public Climber(ClimberIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                kP.initDefault(ClimberConstants.kPReal);
                kI.initDefault(ClimberConstants.kIReal);
                kD.initDefault(ClimberConstants.kDReal);

                kFF.initDefault(ClimberConstants.kFFReal);
                io.setSimpleFF(kFF.get());
                break;

            case REPLAY:
                kP.initDefault(ClimberConstants.kPReplay);
                kI.initDefault(ClimberConstants.kIReplay);
                kD.initDefault(ClimberConstants.kDReplay);

                kFF.initDefault(ClimberConstants.kFFReal);
                break;

            case SIM:
                kP.initDefault(ClimberConstants.kPSim);
                kI.initDefault(ClimberConstants.kISim);
                kD.initDefault(ClimberConstants.kDSim);

                kS.initDefault(ClimberConstants.kSSim);
                kG.initDefault(ClimberConstants.kGSim);
                kV.initDefault(ClimberConstants.kVSim);
                kA.initDefault(ClimberConstants.kASim);
                io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
                break;

            default:
                kP.initDefault(0.0);
                kI.initDefault(0.0);
                kD.initDefault(0.0);

                kFF.initDefault(0.0);
                break;
        }

        io.setPID(kP.get(), kI.get(), kD.get());
        
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs("Climber", inputs);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setSimpleFF(kFF.get()), kFF);
    }

    public void setTargetMeters(double meters) {
        io.setTargetMeters(meters);
        inputs.climberTargetMeters = meters;
    }

    public Command setExtensionCmd(DoubleSupplier meters) {
        return run(() -> setTargetMeters(meters.getAsDouble()));
    }

    public Command runCurrentHoming() {
        return Commands.runOnce(
            () -> io.setVoltage(-1.0)
            )
            .until(() -> inputs.climberCurrentAmps > 40.0)
            .finallyDo(() -> io.resetEncoder(0.0)
            );
    }

    public double getExtensionMeters() {
        return inputs.climberPositionMeters;
      }

    public double getTargetMeters() {
        return inputs.climberTargetMeters;
    }
}