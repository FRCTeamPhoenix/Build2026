package org.team2342.frc.util;

import org.team2342.lib.util.AllianceUtils;
import org.team2342.lib.util.EnhancedXboxController;
import org.team2342.frc.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class PassingController {
    
    private final EnhancedXboxController operatorController;

    private final Translation2d leftTarget = AllianceUtils.flipToAlliance(FieldConstants.LeftBump.nearLeftCorner);
    private final Translation2d rightTarget = AllianceUtils.flipToAlliance(FieldConstants.RightBump.farLeftCorner);

    private Translation2d adjustedLeftTarget = leftTarget;
    private Translation2d adjustedRightTarget = rightTarget;

    private static final double MAX_OFFSET = 0.5; 
    private static final double ADJUST_SCALE = 0.1; 

    private Translation2d leftMin;
    private Translation2d leftMax;

    private Translation2d rightMin;
    private Translation2d rightMax;

    public PassingController(EnhancedXboxController operatorController) {
        this.operatorController = operatorController;

        leftMin = leftTarget.minus(new Translation2d(MAX_OFFSET, MAX_OFFSET));
        leftMax = leftTarget.plus(new Translation2d(MAX_OFFSET, MAX_OFFSET));

        rightMin = rightTarget.minus(new Translation2d(MAX_OFFSET, MAX_OFFSET));
        rightMax = rightTarget.plus(new Translation2d(MAX_OFFSET, MAX_OFFSET));
    }

    public void periodic() {
        double leftX = operatorController.getLeftX();
        double leftY = operatorController.getLeftY();
        Translation2d leftAdjustment =
            DriveCommands.getLinearVelocityFromJoysticks(leftX, leftY).times(ADJUST_SCALE);
        adjustedLeftTarget = adjustedLeftTarget.plus(leftAdjustment);
        adjustedLeftTarget = clamp(adjustedLeftTarget, leftMin, leftMax);

        double rightX = operatorController.getRightX();
        double rightY = operatorController.getRightY();
        Translation2d rightAdjustment = 
            DriveCommands.getLinearVelocityFromJoysticks(rightX, rightY).times(ADJUST_SCALE);
        adjustedRightTarget = adjustedRightTarget.plus(rightAdjustment);
        adjustedRightTarget = clamp(adjustedRightTarget, rightMin, rightMax);
    }

    public Translation2d clamp(Translation2d translation, Translation2d min, Translation2d max) {
        return new Translation2d(
            MathUtil.clamp(translation.getX(), 
            min.getX(), 
            max.getX()), 
            MathUtil.clamp(translation.getY(), 
            min.getY(), 
            max.getY()));    
    }

    public Translation2d getLeftTarget() {
        return adjustedLeftTarget;
    }

    public Translation2d getRightTarget() {
        return adjustedRightTarget;
    }

    public void reset() {
        adjustedLeftTarget = leftTarget;
        adjustedRightTarget = rightTarget;
    }
}