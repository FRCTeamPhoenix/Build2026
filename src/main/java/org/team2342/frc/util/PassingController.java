package org.team2342.frc.util;

import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.EnhancedXboxController;
import org.team2342.frc.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class PassingController {
    
    private final EnhancedXboxController operatorController;

    private Translation2d leftTarget = AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner);
    private Translation2d rightTarget = AllianceUtils.flipToAlliance(FieldConstants.RightBump.farLeftCorner);

    public PassingController(EnhancedXboxController operatorController) {
        this.operatorController = operatorController;
    }

    public void periodic() {
        Translation2d leftJoystick = DriveCommands.getLinearVelocityFromJoysticks(leftTarget.getX(), leftTarget.getY());
        Translation2d rightJoystick = DriveCommands.getLinearVelocityFromJoysticks(rightTarget.getX(), rightTarget.getY());
    }

    public Translation2d clamp(Translation2d translation, Translation2d min, Translation2d max) {
        return new Translation2d(MathUtil.clamp(translation.getX(), min.getX(), max.getX()), MathUtil.clamp(translation.getY(), min.getY(), max.getY()));    
    }
}
