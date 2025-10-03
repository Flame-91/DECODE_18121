package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final Gamepad gamepad;

    public IntakeCommand(Gamepad gamepad, IntakeSubsystem intake) {
        this.intake = intake;
        this.gamepad = gamepad;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (intake.getFlywheelPower() == 0) {
            intake.setFlywheelPower(1);
        } else {
            intake.setFlywheelPower(0);
        }
    }
}
