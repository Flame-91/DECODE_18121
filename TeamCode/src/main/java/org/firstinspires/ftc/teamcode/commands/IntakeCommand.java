package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final GamepadEx gamepad;
    public IntakeCommand(GamepadEx gamepad, IntakeSubsystem intakeSubsystem) {
        this.gamepad = gamepad;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeServoPower(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeServoPower(0);
    }
}
