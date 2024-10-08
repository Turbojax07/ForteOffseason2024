// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.util.LoggedTunableNumber;

/** Add your docs here. */
public class Pivot extends SubsystemBase {
    public PivotIO io;
    public PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private ArmFeedforward pivotFF;

    // private PIDController pivotPID = new PIDController(0.0, 0.0, 0.0);
    private ProfiledPIDController pivotPID = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(ShooterConstants.maxPivotVelocity, ShooterConstants.maxPivotAccel));
    private LoggedTunableNumber kPPivot = new LoggedTunableNumber("Shooter/kPPivot", ShooterConstants.kPPivot);

    public Pivot(PivotIO io) {
      this.io = io;
    
      kPPivot.initDefault(ShooterConstants.kPPivot);
      pivotPID.enableContinuousInput(0, Math.PI * 2);
      pivotPID.setTolerance(ShooterConstants.pivotTolerance);
      pivotPID.setP(kPPivot.getAsDouble());
      pivotFF = new ArmFeedforward(0.0, ShooterConstants.kGPivot, ShooterConstants.kVPivot, ShooterConstants.kAPivot);
    }

  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);
  }

  public Command setPivotTarget(DoubleSupplier radians) {
    return this.run(
      () -> {
        double volts = pivotPID.calculate(inputs.pivotPosition.getRadians(), radians.getAsDouble()) + pivotFF.calculate(radians.getAsDouble(), 0);

        io.setPivotVoltage(MathUtil.clamp(volts, -12, 12));

        inputs.pivotAppliedVolts = volts;
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

  public Command resetEncoder() {
    return this.run(
      () -> {
        io.resetEncoder();
      }
    );
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.setPivotVoltage(-1.0))
        .until(() -> inputs.pivotCurrentAmps > 20.0)
        .finallyDo(() -> io.resetEncoder());
  }
}