// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.util.AllianceUtils;

public class FiringSolver {
  private static FiringSolver instance;

  private static final int ITERATIONS = 5;

  public static final FiringSolution BUMPER_SHOT = new FiringSolution(new Rotation2d(), 0.0, 0.0);

  private FiringSolution lastSolution = null;

  // TODO: Replace with real transform
  public static Transform3d TURRET_TRANSFORM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-4.960),
              Units.inchesToMeters(5.997),
              Units.inchesToMeters(14.823)),
          new Rotation3d(Rotation2d.k180deg));

  private static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  // TODO: tune real maps
  static {
    angleMap.put(1.859, 0.0);
    angleMap.put(2.203, 0.0);
    angleMap.put(2.510, 0.0);
    angleMap.put(2.510, 0.0);
    angleMap.put(2.844, 0.0);
    angleMap.put(3.046, 0.0);
    angleMap.put(3.315, 0.0);
    angleMap.put(3.973, 0.0);
    angleMap.put(5.042, 0.0);

    speedMap.put(1.859, 21.0);
    speedMap.put(2.203, 21.0);
    speedMap.put(2.510, 22.0);
    speedMap.put(2.844, 22.5);
    speedMap.put(3.046, 23.0);
    speedMap.put(3.315, 23.5);
    speedMap.put(3.973, 25.5);
    speedMap.put(5.042, 30.0);

    tofMap.put(1.0, 0.0);
  }

  public static FiringSolver getInstance() {
    if (instance == null) {
      instance = new FiringSolver();
    }
    return instance;
  }

  public FiringSolution calculate(ChassisSpeeds velocity, Pose2d position) {
    if (lastSolution != null) {
      return lastSolution;
    }

    Translation2d hub =
        AllianceUtils.flipToAlliance(FieldConstants.Hub.topCenterPoint).toTranslation2d();
    Pose2d turretPose = new Pose3d(position).transformBy(TURRET_TRANSFORM).toPose2d();
    double robotAngle = turretPose.getRotation().getRadians();

    double velX =
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (TURRET_TRANSFORM.getY() * Math.cos(robotAngle)
                    - TURRET_TRANSFORM.getX() * Math.sin(robotAngle));
    double velY =
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (TURRET_TRANSFORM.getX() * Math.cos(robotAngle)
                    - TURRET_TRANSFORM.getY() * Math.sin(robotAngle));

    double tof;
    Pose2d predictedPose = turretPose;
    double predictedDistance = hub.getDistance(turretPose.getTranslation());
    for (int i = 0; i < ITERATIONS; i++) {
      tof = tofMap.get(predictedDistance);
      predictedPose =
          new Pose2d(
              turretPose.getTranslation().plus(new Translation2d(velX * tof, velY * tof)),
              turretPose.getRotation());
      predictedDistance = hub.getDistance(turretPose.getTranslation());
    }
    Logger.recordOutput("FiringSolver/PredictedPose", predictedPose);
    Logger.recordOutput("FiringSolver/PredictedDistance", predictedDistance);

    Rotation2d turretAngle =
        hub.minus(predictedPose.transformBy(to2d(TURRET_TRANSFORM.inverse())).getTranslation())
            .getAngle()
            .plus(Rotation2d.k180deg);
    double hoodAngle = angleMap.get(predictedDistance);
    double wheelSpeed = speedMap.get(predictedDistance);

    lastSolution = new FiringSolution(turretAngle, wheelSpeed, hoodAngle);

    return lastSolution;
  }

  public void clearCachedSolution() {
    lastSolution = null;
  }

  private Transform2d to2d(Transform3d transform) {
    return new Transform2d(
        new Translation2d(transform.getX(), transform.getY()),
        transform.getRotation().toRotation2d());
  }

  public record FiringSolution(Rotation2d turretAngle, double wheelSpeed, double hoodAngle) {}
}
