package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class FlywheelCommandManual extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final GamepadEx gamepad;

    public FlywheelCommandManual(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .5) {
            flyWheelSubsystem.runFlywheel(.567);
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .5) {
            flyWheelSubsystem.runFlywheel(-.567);
        } else {
            flyWheelSubsystem.runFlywheel(0);
        }

        if (gamepad.getButton(GamepadKeys.Button.A)) {
            flyWheelSubsystem.runFlywheelServos(1.0);
        } else if (gamepad.getButton(GamepadKeys.Button.X)) {
            flyWheelSubsystem.runFlywheelServos(-1.0);
        } else {
            flyWheelSubsystem.runFlywheelServos(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}
