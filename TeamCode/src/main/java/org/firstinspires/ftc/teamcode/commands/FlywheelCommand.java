package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends CommandBase {
    private final FlywheelSubsystem flywheelSubsystem;
    private final GamepadEx gamepad;

    public FlywheelCommand(GamepadEx gamepad, FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.gamepad = gamepad;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void execute() {
        flywheelSubsystem.setServosPower(1);
        flywheelSubsystem.setFlywheelMotor(1);
    }

    @Override
    public boolean isFinished() {
        return !gamepad.getButton(GamepadKeys.Button.Y);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setServosPower(0);
        flywheelSubsystem.setFlywheelMotor(0);
    }
}
