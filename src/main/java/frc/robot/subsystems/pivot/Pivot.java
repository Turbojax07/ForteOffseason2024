// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.util.LoggedTunableNumber;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    public PivotIO io;
    public PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private ArmFeedforward pivotFF;

    private LoggedTunableNumber kPPivot = new LoggedTunableNumber("Shooter/kPPivot");

    public Pivot(PivotIO io) {
      this.io = io;

      switch (Constants.currentMode) {
        case REAL:
          kPPivot.initDefault(ShooterConstants.kPPivotReal);
          break;
        case SIM:
          kPPivot.initDefault(ShooterConstants.kPPivotSim);
          break;
        case REPLAY:
          kPPivot.initDefault(ShooterConstants.kPPivotReplay);
          break;
        default:
          kPPivot.initDefault(0.0);
      }
        io.setPivotPID(kPPivot.getAsDouble(), 0.0, 0.0);
        pivotFF = new ArmFeedforward(0.0, ShooterConstants.kGPivot, ShooterConstants.kVPivot, ShooterConstants.kAPivot);
    }

  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);
  }

  public Command setPivotTarget(DoubleSupplier radians) {
    return this.run(
      () -> {
        io.setPivotTarget(radians.getAsDouble(), pivotFF);
        inputs.pivotTargetPosition = Rotation2d.fromRadians(radians.getAsDouble() + ShooterConstants.simOffset);
      }
    );
  }

  public Command setPivotVoltage(DoubleSupplier volts) {
    return this.run(
      () -> {
        io.setPivotVoltage(volts.getAsDouble());
      }
    );
  }

  public double getAngleRadians() {
    return inputs.pivotPosition.getRadians() + ShooterConstants.simOffset;
  }

  public double getTargetRadians() {
    return inputs.pivotTargetPosition.getRadians();
  }
}