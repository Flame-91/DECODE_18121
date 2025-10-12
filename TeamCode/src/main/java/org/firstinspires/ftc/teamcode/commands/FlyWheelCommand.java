package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class FlyWheelCommand extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final Gamepad gamepad;

    public FlyWheelCommand(Gamepad gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.flyWheelSubsystem = flyWheelSubsystem;
        this.gamepad = gamepad;
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        flyWheelSubsystem.FlyWheelLaunch();
    }

    @Override
    public boolean isFinished() {
        if (!gamepad.a) {
            flyWheelSubsystem.reset();
            return true;
        }
        return false;
    }
}