// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.util.LoggedTunableNumber;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
    public ShooterIO io;
    public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public BeambreakIO feederBeambreak;
    public BeambreakIOInputsAutoLogged feederBeambreakInputs = new BeambreakIOInputsAutoLogged();
    public BeambreakIO shooterBeambreak;
    public BeambreakIOInputsAutoLogged shooterBeambreakInputs = new BeambreakIOInputsAutoLogged();

    private ArmFeedforward pivotFF;
    private SimpleMotorFeedforward shooterFF;

    private LoggedTunableNumber kPPivot = new LoggedTunableNumber("Shooter/kPPivot");

    private LoggedTunableNumber kPShooter = new LoggedTunableNumber("Shooter/kPShooter");
    private LoggedTunableNumber kSShooter = new LoggedTunableNumber("Shooter/kSShooter");


    public Shooter(ShooterIO io, BeambreakIO feederBeambreak, BeambreakIO shooterBeambreak) {
      this.io = io;
      this.feederBeambreak = feederBeambreak;
      this.shooterBeambreak = shooterBeambreak;

      switch (Constants.currentMode) {
        case REAL:
          kPPivot.initDefault(ShooterConstants.kPPivotReal);

          kPShooter.initDefault(ShooterConstants.kPShooterReal);
          kSShooter.initDefault(ShooterConstants.kSShooterReal);
          break;
        case SIM:
          kPPivot.initDefault(ShooterConstants.kPPivotSim);

          kPShooter.initDefault(ShooterConstants.kPShooterSim);
          kSShooter.initDefault(ShooterConstants.kSShooterSim);
          break;
        case REPLAY:
          kPPivot.initDefault(ShooterConstants.kPPivotReplay);

          kPShooter.initDefault(ShooterConstants.kPShooterReplay);
          kSShooter.initDefault(ShooterConstants.kSShooterReplay);
          break;
        default:
          kPPivot.initDefault(0.0);

          kPShooter.initDefault(0.0);
          kSShooter.initDefault(0.0);
      }
        io.setPivotPID(kPPivot.getAsDouble(), 0.0, 0.0);
        pivotFF = new ArmFeedforward(0.0, ShooterConstants.kGPivot, ShooterConstants.kVPivot, ShooterConstants.kAPivot);

        io.setShooterPID(kPShooter.getAsDouble(), 0.0, 0.0);
        shooterFF = new SimpleMotorFeedforward(kSShooter.getAsDouble(), ShooterConstants.kVShooter, ShooterConstants.kAShooter);
  }

  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command setPivotTarget(DoubleSupplier radians) {
    return this.run(
      () -> {
        io.setPivotTarget(radians.getAsDouble(), pivotFF);
        inputs.pivotTargetPosition = Rotation2d.fromRadians(radians.getAsDouble() + ShooterConstants.simOffset);
      }
    );
  }

  public Command setLeftRPM(IntSupplier rpm) {
    return this.run(
      () -> {
        io.setLeftRPM(rpm.getAsInt(), shooterFF);
        inputs.leftTargetRPM = rpm.getAsInt();
      }
    );
  }

  public Command setLeftVoltage(DoubleSupplier volts) {
    return this.run(
      () -> {
        io.setLeftVoltage(volts.getAsDouble());
      }
    );
  }

  public Command setRightVoltage(DoubleSupplier volts) {
    return this.run(
      () -> {
        io.setRightVoltage(volts.getAsDouble());
      }
    );
  }

  public Command setRightRPM(IntSupplier rpm) {
    return this.run(
      () -> {
        io.setRightRPM(rpm.getAsInt(), shooterFF);
        inputs.rightTargetRPM = rpm.getAsInt();
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
