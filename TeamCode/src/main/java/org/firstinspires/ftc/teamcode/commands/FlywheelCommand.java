package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends CommandBase {
    private final FlywheelSubsystem flywheelSubsystem;
    private final Gamepad gamepad;
    public FlywheelCommand(Gamepad gamepad, FlywheelSubsystem flywheelSubsystem) {
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
        return !gamepad.y;
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setServosPower(0);
        flywheelSubsystem.setFlywheelMotor(0);
    }
}
