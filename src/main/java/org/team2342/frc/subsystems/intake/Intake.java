// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

public class Intake extends SubsystemBase {
  private final DumbMotorIO intakeMotor;
  private final DumbMotorIOInputsAutoLogged intakeMotorInputs = new DumbMotorIOInputsAutoLogged();

  public Intake(DumbMotorIO intakeMotor) {
    this.intakeMotor = intakeMotor;
    setName("Intake");
    setDefaultCommand(run(() -> intakeMotor.runVoltage(0.0)));
  }

  @Override
  public void periodic() {
    intakeMotor.updateInputs(intakeMotorInputs);
    Logger.processInputs("Intake/IntakeMotor", intakeMotorInputs);
  }

  public Command runIntake(double voltage) {

    return run(() -> intakeMotor.runVoltage(voltage)).withName("Run Intake");
  }

  public Command stopIntake() {

    return runOnce(() -> intakeMotor.runVoltage(0)).withName("Stop Intake");
  }
}
