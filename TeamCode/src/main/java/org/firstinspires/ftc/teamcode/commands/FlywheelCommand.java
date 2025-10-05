package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.RefCounted;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

public class FlywheelCommand extends CommandBase {
    private final FlywheelSubsystem flywheelSubsystem;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    public FlywheelCommand(Gamepad gamepad, FlywheelSubsystem flywheelSubsystem, Telemetry telemetry) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    @Override
    public void execute() {
        flywheelSubsystem.setServosPower(1);
        flywheelSubsystem.setFlywheelMotor(1);
        telemetry.addData("servos & flywheel", "on");
    }

    @Override
    public boolean isFinished() {
        return !gamepad.y;
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setServosPower(0);
        flywheelSubsystem.setFlywheelMotor(0);
        telemetry.addData("servos & flywheel", "on");
    }
}
