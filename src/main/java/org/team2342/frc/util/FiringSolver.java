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
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.util.AllianceUtils;

public class FiringSolver {
  private static final int ITERATIONS = 5;

  // TODO: Replace with real transform
  private Transform3d turretTransform = new Transform3d();

  private static final InterpolatingTreeMap<Double, Rotation2d> angleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  static {
    // Add to tables here
  }

  public FiringSolution calculate(ChassisSpeeds velocity, Pose2d position) {
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
    Rotation2d hoodAngle = angleMap.get(predictedDistance);
    double wheelSpeed = speedMap.get(predictedDistance);

    return new FiringSolution(turretAngle, wheelSpeed, hoodAngle);
  }

  private record FiringSolution(Rotation2d turretAngle, double wheelSpeed, Rotation2d hoodAngle) {}
}
