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

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean isTuning = false;

  public static class RobotMap {
    public static class Drive {
      public static final int frontLeftDrive = 1;
      public static final int frontLeftTurn = 2;
      public static final int frontRightDrive = 3;
      public static final int frontRightTurn = 4;
      public static final int backLeftDrive = 5;
      public static final int backLeftTurn = 6;
      public static final int backRightDrive = 7;
      public static final int backRightTurn = 8;

      public static final boolean frontLeftTurnInvert = false;
      public static final boolean frontRightTurnInvert = false;
      public static final boolean backLeftTurnInvert = false;
      public static final boolean backRightTurnInvert = false;

      public static final int frontLeftEncoder = 2;
      public static final int frontRightEncoder = 3;
      public static final int backLeftEncoder = 1;
      public static final int backRightEncoder = 0;

      public static final double frontLeftOffset = .749787;
      public static final double frontRightOffset = -0.509314;
      public static final double backLeftOffset = .173251 + 0.914402;
      public static final double backRightOffset = 0.093198;

      public static final int gyro = 0;
    }

    public static class Intake {
      public static final int pivot = 11;
      public static final int roller = 12;
    }

    public static class Shooter {
      public static final int feeder = 21;
      public static final int pivot = 22;
      public static final int shooterLeft = 23;
      public static final int shooterRight = 24;

      public static final int feederBeambreak = 0;
      public static final int shooterBeambreak = 1;
    }

    public static class Climber {
      public static final int climber = 31;
    }
    
  }

  public static class ControlConstants {
    public static final double deadband = 0.01;
  }

  public static class DriveConstants {
    public static final double trackWidth = Units.inchesToMeters(19.5);
    public static final double wheelRadius = Units.inchesToMeters(2);

    public static final double driveRatio = 5.14;
    public static final double driveInertia = 0.025;
    public static final double turnRatio = 12.8;
    public static final double turnInertia = 0.004;

    public static final double driveConversion = (driveRatio) * (1.0 / (wheelRadius * 2 * Math.PI));
    public static final double turnConversion = 2 * Math.PI / turnRatio;
    public static final double turnVelocityConversion = turnConversion / 60;

    public static final int driveSupplyCurrent = 10; // 70
    public static final int driveStatorCurrent = 15; // 120
    public static final int turnCurrent = 10; // 30

    public static final double odometeryFrequency = 250;
    public static final double updateFrequency = 100;

    // public static final double maxLinearVelocity = Units.feetToMeters(20.4);
     public static final double maxLinearVelocity = Units.feetToMeters(1);
    public static final double maxLinearAccel = 8.0;

    public static final double maxAngularVelocity = maxLinearVelocity / (Math.hypot(trackWidth / 2.0, trackWidth / 2.0));
    public static final double maxAngularAccel = maxLinearAccel / (Math.hypot(trackWidth / 2.0, trackWidth / 2.0));

    public static double kPDriveReal = 2.0;
    public static double kDDriveReal = 0.2;
    public static double kSDriveReal = 0.04;
    public static double kVDriveReal = 2.381;
    public static double kADriveReal = 0.65;

    public static double kPTurnReal = .1; // 1.5?
    public static double kDTurnReal = 0.0;

    public static double kPDriveSim = 0.3;
    public static double kDDriveSim = 0.0;
    public static double kSDriveSim = 0.0;
    public static double kVDriveSim = 2.0;
    public static double kADriveSim = 0.0;

    public static double kPTurnSim = 100.0;
    public static double kDTurnSim = 0.0;

    public static double kPDriveReplay = 0.0;
    public static double kDDriveReplay = 0.0;
    public static double kSDriveReplay = 0.0;
    public static double kVDriveReplay = 0.0;
    public static double kADriveReplay = 0.0;
    
    public static double kPTurnReplay = 0.0;
    public static double kDTurnReplay = 0.0;
  }

  public static class ClimberConstants {
    public static double gearRatio = 45;
    public static double spoolRadius = Units.inchesToMeters(.75);
    public static double encoderConversion = 2 * spoolRadius * Math.PI / gearRatio;
    public static double width = Units.inchesToMeters(2.0);

    public static double minHeight = 0.0;
    public static double maxHeight = Units.inchesToMeters(18);

    public static double maxVelocity = Units.inchesToMeters(17);
    public static double maxAccel = Units.inchesToMeters(180);

    public static double kPSim = 20;
    public static double kISim = 0.0;
    public static double kDSim = 0.0;

    public static double kPReal = 0.0;
    public static double kIReal = 0.0;
    public static double kDReal = 0.0;

    public static double kPReplay = 0.0;
    public static double kIReplay = 0.0;
    public static double kDReplay = 0.0;

    public static double kFFSim = 0.0;
    public static double kFFReal = 0.0;
    public static double kFFReplay = 0.0;

    public static double kSSim = 0.0;
    public static double kGSim = 0.0;
    public static double kVSim = 0.0;
    public static double kASim = 0.0;
}

  public static class IntakeConstants {
    public static double kPPivotReal = 0.01;
    public static double kIPivotReal = 0.0;
    public static double kDPivotReal = 0.0;

    public static double kPRollerReal = 1.5;
    public static double kIRollerReal = 0.0;
    public static double kDRollerReal = 0.0;
    public static double kFFRollerReal = 0.0;

    public static double kPPivotSim = 0.3;
    public static double kIPivotSim = 0.0;
    public static double kDPivotSim = 0.0;

    public static double kPRollerSim = 10;
    public static double kIRollerSim = 0.0;
    public static double kDRollerSim = 0.0;
    public static double kFFRollerSim = 0.0;

    public static double kPPivotReplay = 0.3;
    public static double kIPivotReplay = 0.0;
    public static double kDPivotReplay = 0.0;

    public static double kPRollerReplay = 10;
    public static double kIRollerReplay = 0.0;
    public static double kDRollerReplay = 0.0;
    public static double kFFRollerReplay = 0.0;
  }
  public static class SimConstants {
    public static final double loopTime = 0.02;
  }
}
