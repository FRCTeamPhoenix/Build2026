// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.util.AllianceUtils;

public class FiringSolver {
  private static FiringSolver instance;

  private static final int ITERATIONS = 5;

  public static final FiringSolution BUMPERSHOT = new FiringSolution(new Rotation2d(), 1.0, 1.0);

  private FiringSolution lastSolution = null;

  // TODO: Replace with real transform
  private Transform3d turretTransform = new Transform3d();

  private static final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  static {
    // Add to tables here
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
    Pose2d turretPose = new Pose3d(position).transformBy(turretTransform).toPose2d();
    double robotAngle = turretPose.getRotation().getRadians();

    double velX =
        velocity.vxMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (turretTransform.getY() * Math.cos(robotAngle)
                    - turretTransform.getX() * Math.sin(robotAngle));
    double velY =
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (turretTransform.getX() * Math.cos(robotAngle)
                    - turretTransform.getY() * Math.sin(robotAngle));

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

    Rotation2d turretAngle = hub.minus(predictedPose.getTranslation()).getAngle();
    double hoodAngle = angleMap.get(predictedDistance);
    double wheelSpeed = speedMap.get(predictedDistance);

    lastSolution = new FiringSolution(turretAngle, wheelSpeed, hoodAngle);

    return lastSolution;
  }

  public void clearCachedSolution() {
    lastSolution = null;
  }

  public record FiringSolution(Rotation2d turretAngle, double wheelSpeed, double hoodAngle) {}
}
