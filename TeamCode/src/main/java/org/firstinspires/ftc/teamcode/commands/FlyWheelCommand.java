package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class FlyWheelCommand extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;

    public FlyWheelCommand(FlyWheelSubsystem flyWheelSubsystem) {
        this.flyWheelSubsystem = flyWheelSubsystem;
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        flyWheelSubsystem.FlyWheelLaunch();
    }
}