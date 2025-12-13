package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private boolean on;
    private GamepadEx gamepadEx;
    public IntakeCommand(GamepadEx gamepadEx, IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.gamepadEx = gamepadEx;
        this.on = false;

        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void execute() {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            on = !on;
        }
        if (on) {
            intakeSubsystem.setIntakeServoPower(1);
        } else {
            intakeSubsystem.setIntakeServoPower(0);
        }
    }
}
