// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.IntakeConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Wheels extends SubsystemBase {
  private final DumbMotorIO intakeMotor;
  private final DumbMotorIOInputsAutoLogged intakeMotorInputs = new DumbMotorIOInputsAutoLogged();

  private final Alert intakeMotorAlert = new Alert("Wheels are disconnected!", AlertType.kError);

  public Wheels(DumbMotorIO intakeMotor) {
    this.intakeMotor = intakeMotor;
    setName("Intake/Wheels");
    setDefaultCommand(run(() -> intakeMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    intakeMotor.updateInputs(intakeMotorInputs);
    Logger.processInputs("Intake/Wheels", intakeMotorInputs);
    intakeMotorAlert.set(!intakeMotorInputs.motorsConnected[0]);
    ExecutionLogger.log("Intake/Wheels");
  }

  public Command runIntake(double voltage) {
    return run(() -> intakeMotor.runVoltage(voltage)).withName("Run Intake Wheels");
  }

  public Command in() {
    return runIntake(IntakeConstants.RUN_VOLTAGE);
  }

  public Command out() {
    return runIntake(-IntakeConstants.RUN_VOLTAGE);
  }

  public Command stopIntake() {
    return runOnce(() -> intakeMotor.runVoltage(0)).withName("Stop Intake Wheels");
  }
}
