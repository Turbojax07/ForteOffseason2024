// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  Timer timeExtended = new Timer();

  private LoggedTunableNumber kPPivot = new LoggedTunableNumber("Intake/kPPivot");
  private LoggedTunableNumber kIPivot = new LoggedTunableNumber("Intake/kIPivot");
  private LoggedTunableNumber kDPivot = new LoggedTunableNumber("Intake/kDPivot");

  private LoggedTunableNumber kPRoller = new LoggedTunableNumber("Intake/kPRoller");
  private LoggedTunableNumber kIRoller = new LoggedTunableNumber("Intake/kIRoller");
  private LoggedTunableNumber kDRoller = new LoggedTunableNumber("Intake/kDRoller");

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    timeExtended.start();
  }

  @Override
  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command setAngleCmd(DoubleSupplier degrees) {
          return this.run(
              () -> {
                  io.setTargetDegrees(degrees.getAsDouble());
                  inputs.pivotTargetDegrees = degrees.getAsDouble();
              }
          );
      }

  public Command setRollerPercent(DoubleSupplier percent) {
    return this.run(
      () -> {
        io.setRollerPercent(percent.getAsDouble());
        inputs.rollerSpeedRPM = percent.getAsDouble() * 5880;
      }
    );
  }

  public double getAngleDegrees() {
    return inputs.pivotAngleDegrees;
  }

  public double getTargetDegrees() {
    return inputs.pivotTargetDegrees;
  }
}
