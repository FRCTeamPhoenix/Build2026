// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Pivot extends SubsystemBase {

  private final SmartMotorIO pivotMotor;
  private final SmartMotorIOInputsAutoLogged pivotMotorInputs = new SmartMotorIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Pivot/TargetAngle")
  private double goal = 2.23;

  private final Alert pivotMotorAlert = new Alert("Pivot motor is disconnected!", AlertType.kError);

  public Pivot(SmartMotorIO pivotMotor) {
    this.pivotMotor = pivotMotor;
    setName("Intake/Pivot");
    setDefaultCommand(run(() -> pivotMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(pivotMotorInputs);
    Logger.processInputs("Intake/Pivot", pivotMotorInputs);
    pivotMotorAlert.set(!pivotMotorInputs.motorsConnected[0]);
    ExecutionLogger.log("Intake/Pivot");
  }

  public void runAngle(double angle) {
    goal = MathUtil.clamp(angle, IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE);
    pivotMotor.runPosition(goal);
  }

  public Command goToAngle(double angle) {
    return run(() -> runAngle(angle))
        .until(() -> Math.abs(goal - angle) <= 0.01)
        .withName("Pivot Go To Angle");
  }

  public Command runVoltage(double voltage) {
    return run(() -> pivotMotor.runVoltage(voltage)).withName("Pivot Run Voltage");
  }

  public Command holdAngle(double angle) {
    return run(() -> runAngle(angle)).withName("Pivot Hold Angle");
  }

  public Command stop() {
    return runOnce(() -> pivotMotor.runVoltage(0.0)).withName("Pivot Stop");
  }
}
