package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeServoPower(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
