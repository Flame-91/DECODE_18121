package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class ServoCommand extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;
    private Gamepad gamepad;

    public ServoCommand( Gamepad gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.flyWheelSubsystem = flyWheelSubsystem;
        this.gamepad = gamepad;
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        flyWheelSubsystem.feed();
    }

    @Override
    public boolean isFinished() {
        if (!gamepad.x) {
            flyWheelSubsystem.resetFeed();
            return true;
        } else {
            return false;
        }
    }
}
