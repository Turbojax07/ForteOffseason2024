package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.util.LoggedTunableNumber;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP");
    private LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI");
    private LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD");

    private LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS");
    private LoggedTunableNumber kG = new LoggedTunableNumber("Climb/kG");
    private LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV");
    private LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA");
    private LoggedTunableNumber kFF = new LoggedTunableNumber("Climb/kFF");

    public Climb(ClimbIO io) {
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
                kP.initDefault(ClimbConstants.kPReal);
                kI.initDefault(ClimbConstants.kIReal);
                kD.initDefault(ClimbConstants.kDReal);

                kFF.initDefault(ClimbConstants.kFFReal);
                io.setSimpleFF(kFF.get());
                break;

            case REPLAY:
                kP.initDefault(ClimbConstants.kPReplay);
                kI.initDefault(ClimbConstants.kIReplay);
                kD.initDefault(ClimbConstants.kDReplay);

                kFF.initDefault(ClimbConstants.kFFReal);
                break;

            case SIM:
                kP.initDefault(ClimbConstants.kPSim);
                kI.initDefault(ClimbConstants.kISim);
                kD.initDefault(ClimbConstants.kDSim);

                kS.initDefault(ClimbConstants.kSSim);
                kG.initDefault(ClimbConstants.kGSim);
                kV.initDefault(ClimbConstants.kVSim);
                kA.initDefault(ClimbConstants.kASim);
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
        Logger.processInputs("Climb", inputs);

        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setSimpleFF(kFF.get()), kFF);
    }

    public void setTargetMeters(double meters) {
        io.setTargetMeters(meters);
        inputs.climberTargetMeters = meters;
    }


    public Command setDutyCycle(double dutyCycle) {
        return run(() -> {
            io.setOpenLoopDutyCycle(dutyCycle);
            inputs.climberTargetSpeed = dutyCycle;
        });
    }

    public Command setExtensionCmd(DoubleSupplier meters) {
        return run(() -> setTargetMeters(meters.getAsDouble()));
    }

    public Command runCurrentHoming() {
        return Commands.runOnce(
            () -> io.setVoltage(-1.0)
        ).until(
            () -> inputs.climberCurrentAmps > 40.0
        ).finallyDo(
            () -> io.resetEncoder(0.0)
        );
    }

    public double getExtensionMeters() {
        return inputs.climberPositionMeters;
      }

    public double getTargetMeters() {
        return inputs.climberTargetMeters;
    }
}