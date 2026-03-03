package org.team2342.frc.subsystems.kicker;

import org.littletonrobotics.junction.Logger;
import org.team2342.frc.Constants.KickerConstants;
import org.team2342.lib.logging.ExecutionLogger;
import org.team2342.lib.motors.dumb.DumbMotorIO;
import org.team2342.lib.motors.dumb.DumbMotorIOInputsAutoLogged;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase{
    private final DumbMotorIO kickerMotor;
    
    private final DumbMotorIOInputsAutoLogged kickerMotorInputs = new DumbMotorIOInputsAutoLogged();
    private final Alert kickerMotorAlert =
      new Alert("Indexer Feeder Motor is diconnected", AlertType.kError);

    public Kicker(DumbMotorIO kickerMotor) {
        this.kickerMotor = kickerMotor;
        setName("Shooter/Kicker");
        setDefaultCommand(run(() -> kickerMotor.runVoltage(0.0)));
    }

    @Override
    public void periodic() {
        kickerMotor.updateInputs(kickerMotorInputs);
        
        Logger.processInputs("Shooter/Kicker", kickerMotorInputs);

        kickerMotorAlert.set(!kickerMotorInputs.connected);

        ExecutionLogger.log("Shooter/Kicker");
    }

    public Command in() {
        return run(() -> kickerMotor.runVoltage(KickerConstants.RUN_VOLTAGE)).withName("Kicker Motor Run");
    }
    public Command out() {
        return run(() -> kickerMotor.runVoltage(-KickerConstants.RUN_VOLTAGE)).withName("Kicker Motor Run");
    }
    public Command stop() {
        return runOnce(
                () -> {
                    kickerMotor.runVoltage(0.0);
                })
            .withName("Kicker Stop");
    }
}
