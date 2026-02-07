// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class LauncherVariables {
  public static Transform3d robotToTurret = new Transform3d(0, 0.0, 0, Rotation3d.kZero); //TODO: change these
  public static Transform3d turretToCamera =
      new Transform3d(
          0, 0.0, 0, new Rotation3d(0.0, Units.degreesToRadians(0), 0.0));

  private LauncherVariables() {}
}
