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
        double rightTrigger = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean a = gamepad.getButton(GamepadKeys.Button.A);
        boolean x = gamepad.getButton(GamepadKeys.Button.X);
        if (rightTrigger > .5)
            flyWheelSubsystem.runFlywheel(.35);
        if (a)
            flyWheelSubsystem.runFlywheelServos(1.0);
        else if (x)
            flyWheelSubsystem.runFlywheelServos(-1.0);
        else
            flyWheelSubsystem.runFlywheelServos(0);
    }

    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}
