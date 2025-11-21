package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final GamepadEx gamepad;
    private enum Running {RUN, STOP}
    private Running running = Running.RUN;
    public IntakeCommand(GamepadEx gamepad, IntakeSubsystem intakeSubsystem) {
        this.gamepad = gamepad;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            switch (running) {
                case RUN:
                    running = Running.STOP;
                    break;
                case STOP:
                    running = Running.RUN;
                    break;
            }
        }
        switch (running) {
            case RUN:
                intakeSubsystem.setIntakeServoPower(1.0);
                break;
            case STOP:
                intakeSubsystem.setIntakeServoPower(0);
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeServoPower(0);
    }
}
