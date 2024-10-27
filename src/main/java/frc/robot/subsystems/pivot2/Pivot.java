// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot2;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
  public PivotIO io;
  public PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  boolean overshoot;

  private ArmFeedforward pivotFF =
      new ArmFeedforward(
          0.0, ShooterConstants.kGPivot, ShooterConstants.kVPivot, ShooterConstants.kAPivot);
  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          0.0,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              ShooterConstants.maxPivotVelocity, ShooterConstants.maxPivotAccel));
  private LoggedTunableNumber kPPivot =
      new LoggedTunableNumber("Shooter/kPPivot", ShooterConstants.kPPivot);

  public Pivot(PivotIO io) {
    this.io = io;

    kPPivot.initDefault(ShooterConstants.kPPivot);
    pivotPID.enableContinuousInput(0, Math.PI * 2);
    pivotPID.setTolerance(ShooterConstants.pivotTolerance);
    pivotPID.setP(kPPivot.getAsDouble());
  }

  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);
    overshoot =
        (inputs.pivotPosition.getRadians() > inputs.pivotTargetPosition.getRadians())
            && (inputs.pivotTargetPosition.getRadians() != 0);

    // if (overshoot) {
    //   io.setPivotVoltage(
    //       pivotPID.calculate(
    //           inputs.pivotPosition.getRadians(), inputs.pivotTargetPosition.getRadians()));
    // }
    Logger.recordOutput("Test/Overshoot", overshoot);
  }

  public Command setPivotTarget(DoubleSupplier radians) {
    return this.run(
        () -> {
          double volts =
              pivotPID.calculate(inputs.pivotPosition.getRadians(), radians.getAsDouble())
                  + pivotFF.calculate(
                      pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);

          io.setPivotVoltage(volts);
          inputs.pivotAppliedVolts = volts;
          inputs.pivotTargetPosition = Rotation2d.fromRadians(radians.getAsDouble());
          Logger.recordOutput("Test/positionSetpoint", radians.getAsDouble());
          Logger.recordOutput("Test/position", inputs.pivotPosition.getRadians());
        });
  }

  public Command setPivotVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setPivotVoltage(volts.getAsDouble());
          inputs.pivotAppliedVolts = volts.getAsDouble();
        });
  }

  public double getAngleRadians() {
    return inputs.pivotPosition.getRadians() + ShooterConstants.simOffset;
  }

  public double getTargetRadians() {
    return inputs.pivotTargetPosition.getRadians() + ShooterConstants.simOffset;
  }

  public Command resetEncoder() {
    return this.run(
        () -> {
          io.resetEncoder();
        });
  }

  public double getRelativeRadians() {
    return inputs.pivotRelativeEncoder.getRadians() + ShooterConstants.simOffset;
  }

  public boolean atSetpoint() {
    return pivotPID.atSetpoint();
  }

  public boolean isStalled() {
    return inputs.pivotStalled;
  }

  public Command runZero() {
    return this.run(
            () -> {
              io.setPivotVoltage(-1);
            })
        .until(() -> inputs.pivotStalled)
        .finallyDo(() -> io.resetEncoder());
  }
}
