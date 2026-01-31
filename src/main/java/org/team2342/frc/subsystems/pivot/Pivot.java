package org.team2342.frc.subsystems.pivot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Pivot extends SubsystemBase {
    
    private final SmartMotorIO pivotMotor;
    private final SmartMotorIOInputsAutoLogged pivotMotorInputs = new SmartMotorIOInputsAutoLogged();

    public Pivot(SmartMotorIO pivotMotor) {
        this.pivotMotor = pivotMotor;
        setName("Pivot");
        setDefaultCommand(run(() -> pivotMotor.runVoltage(0.0)));

    @Override
    public void periodic() {
        pivotMotor.updateInputs(pivotMotorInputs);
        Logger.processInputs("Pivot/pivotMotor", pivotMotorInputs);
    }
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(pivotMotorInputs.positionRad / PivotConstants.GEAR_RATIO);
  }

    public Command goToAngle(Rotation2d targetAngle) {
        return run(() -> pivotMotor.runPosition(targetAngle.getRadians()))
            .until(() -> {
                // Using minus() and getRadians() handles the wrap-around math for you!
                double error = targetAngle.minus(getAngle()).getRadians();
                return Math.abs(error) < PivotConstants.AT_TARGET_TOLERANCE;
            })
            .withName("Wrist Go To Angle"); // Fixed the quote here
    }

    public Command holdAngle(Rotation2d targetAngle) {
        return run(() -> pivotMotor.runPosition(targetAngle.getRadians())).withName("Wrist Hold Angle");
  }

    public Command shoot(double metersPerSec) {
        double radPerSec = metersPerSec / PivotConstants.WHEEL_RADIUS_METERS;
        return run(() -> pivotMotor.runVelocity(radPerSec)).withName("Run Pivot");
    }

    public Command stop() {
        return runOnce(() -> pivotMotor.runVoltage(0.0)).withName("Stop Stop");
    }

    public double getVelocityMetersPerSec() {
        return pivotMotorInputs.velocityRadPerSec * PivotConstants.WHEEL_RADIUS_METERS;
    }
}

