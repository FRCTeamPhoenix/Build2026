// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.team2342.frc.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2342.lib.motors.smart.SmartMotorIO;
import org.team2342.lib.motors.smart.SmartMotorIOInputsAutoLogged;

public class Turret extends SubsystemBase {

    private final SmartMotorIO turretMotor;
    private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();
    private Rotation2d setpoint = new Rotation2d();
    private double threshold = 0.01;

    public Turret(SmartMotorIO turretMotor) {
        this.turretMotor = turretMotor;
        setName("Turret");
        setDefaultCommand(run(() -> turretMotor.runVoltage(0.0)));
    }

    public Command runPosition(Rotation2d setpoint) {
        this.setpoint = setpoint;
        return run(() -> turretMotor.runPosition(setpoint.getRadians())).withName("turret run position");
    }

    public Command goToPosition(Rotation2d setpoint) {
        this.setpoint = setpoint;
        return run(() -> turretMotor.runPosition(setpoint.getRadians())).until(() -> Math.abs(turretPosition().minus(setpoint).getRadians()) <= threshold).withName("turret go to position");
    }


    public Command stop() {
        return runOnce(() -> turretMotor.runVoltage(0.0));
    }

    public Rotation2d turretPosition() {
        return Rotation2d.fromRadians(inputs.positionRad);
    }

    public Rotation2d turretSetpoint() {
        return setpoint;
    }

    

}
