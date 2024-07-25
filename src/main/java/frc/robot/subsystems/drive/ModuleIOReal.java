// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;

import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOReal implements ModuleIO {
  private final String name;

  private final TalonFX driveTalon;
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final CANSparkMax turnSparkMax;

  private final boolean turnInverted;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogEncoder turnAbsoluteEncoder;
  private final SparkPIDController turnController;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final VelocityVoltage driveCurrentVelocity = new VelocityVoltage(0.0).withEnableFOC(false);

  private final Queue<Double> turnPositionQueue;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOReal(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(RobotMap.Drive.frontLeftDrive);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontLeftTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.frontLeftTurnInvert;
        turnAbsoluteEncoder = new AnalogEncoder(RobotMap.Drive.frontLeftEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.frontLeftOffset); // MUST BE CALIBRATED
        name = "FrontLeft";
        break;
      case 1:
        driveTalon = new TalonFX(RobotMap.Drive.frontRightDrive);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontRightTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.frontRightTurnInvert;
        turnAbsoluteEncoder = new AnalogEncoder(RobotMap.Drive.frontRightEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.frontRightOffset); // MUST BE CALIBRATED
        name = "FrontRight";
        break;
      case 2:
        driveTalon = new TalonFX(RobotMap.Drive.backLeftDrive);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backLeftTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.backLeftTurnInvert;
        turnAbsoluteEncoder = new AnalogEncoder(RobotMap.Drive.backLeftEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.backLeftOffset); // MUST BE CALIBRATED
        name = "BackLeft";
        break;
      case 3:
        driveTalon = new TalonFX(RobotMap.Drive.backRightDrive);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backRightTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.backRightTurnInvert;
        turnAbsoluteEncoder = new AnalogEncoder(RobotMap.Drive.backRightEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.backRightOffset); // MUST BE CALIBRATED
        name = "BackRight";
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveSupplyCurrent;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveStatorCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    driveConfig.MotorOutput.PeakReverseDutyCycle = -1.0;

    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveConversion;
    driveTalon.setPosition(0);

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = DriveConstants.maxLinearVelocity;
    driveConfig.MotionMagic.MotionMagicAcceleration = DriveConstants.maxLinearAccel;
    driveConfig.MotionMagic.MotionMagicJerk = DriveConstants.maxLinearVelocity / 0.1;
    
    driveConfig.Slot0.kP = DriveConstants.kPDriveReal;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = DriveConstants.kDDriveReal;

    driveConfig.Slot0.kS = DriveConstants.kSDriveReal;
    driveConfig.Slot0.kV = DriveConstants.kVDriveReal;
    driveConfig.Slot0.kA = DriveConstants.kADriveReal;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnController = turnSparkMax.getPIDController();

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);

    for (int i = 0; i < 30; i ++) {
      turnController.setFeedbackDevice(turnRelativeEncoder);
      turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI);
      turnRelativeEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);
      
      turnSparkMax.setInverted(turnInverted);
      turnSparkMax.setSmartCurrentLimit(30);
      turnSparkMax.enableVoltageCompensation(12.0);

      turnRelativeEncoder.setPosition(0.0);
      turnRelativeEncoder.setMeasurementPeriod(10);
      turnRelativeEncoder.setAverageDepth(2);

      turnSparkMax.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, (int) (1000.0 / DriveConstants.odometeryFrequency));
    }

    turnController.setPositionPIDWrappingEnabled(true);
    turnController.setPositionPIDWrappingMinInput(-180);
    turnController.setPositionPIDWrappingMaxInput(180);

    turnSparkMax.burnFlash();
    turnSparkMax.setCANTimeout(0);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometeryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    driveTalon.optimizeBusUtilization();

    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DriveConstants.driveRatio;
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DriveConstants.driveRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
      Rotation2d.fromRotations(turnAbsoluteEncoder.getAbsolutePosition()).minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / DriveConstants.turnRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / DriveConstants.turnRatio;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DriveConstants.driveRatio)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.turnRatio))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveVolts(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void runTurnVolts(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public String getModuleName() {
    return name;
  }

  @Override
  public void runDriveVelocitySetpoint(final double metersPerSecond, final double metersPerSecondSquared) {
    // Doesnt actually refresh drive velocity signal, but should be cached
    if (metersPerSecond == 0
        && metersPerSecondSquared == 0
        && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      runDriveVolts(0.0);
    } else {
      driveTalon.setControl(
          driveCurrentVelocity.withVelocity(metersPerSecond).withFeedForward(metersPerSecondSquared * driveConfig.Slot0.kA));
    }
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    // inputs.targetPosition = getAdjustedAngle(angleRads);
    turnController.setReference(angleRads, ControlType.kPosition, 0);
  }

  @Override
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveConfig.Slot0.kS = kS;
    driveConfig.Slot0.kV = kV;
    driveConfig.Slot0.kA = kA;
    driveTalon.getConfigurator().apply(driveConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnController.setP(kP);
    turnController.setI(kI);
    turnController.setD(kD);
    turnSparkMax.burnFlash();
  }

  public void initializeEncoder() {
    turnRelativeEncoder.setPosition((turnAbsoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset.getDegrees()) * (2.0 * Math.PI));
}

  public double getHeading() {
    return turnRelativeEncoder.getPosition()%(2.0 * Math.PI);
  }

  public double getAdjustedAngle(double angle) {
    double theta = getHeading() - angle;

    if (theta >= Math.PI) {
        theta-=(2.0 * Math.PI);
    }
    if (theta <= -Math.PI) {
        theta+=(2.0 * Math.PI);
    }

    return turnRelativeEncoder.getPosition() - theta;
  }

  @Override
  public void stop() {
    var driveRequest = driveTalon.getAppliedControl();
        if(driveRequest instanceof VoltageOut) {
            driveTalon.setControl(new NeutralOut());
        }
        runTurnVolts(0);
  }

}
