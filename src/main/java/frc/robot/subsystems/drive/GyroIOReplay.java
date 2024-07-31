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

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOReplay implements GyroIO {

  public void processInputs(GyroIOInputsAutoLogged inputs) {}

  @Override
  public void setYaw(double yaw) {}

  @Override
  public Rotation2d getYaw() {
    return new Rotation2d();
  }

  @Override
  public void reset() {}
}
