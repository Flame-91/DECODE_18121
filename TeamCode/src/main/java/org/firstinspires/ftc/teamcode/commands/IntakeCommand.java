package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final GamepadEx gamepad;
    private boolean innerFlywheelRunning;
    public IntakeCommand(GamepadEx gamepad, IntakeSubsystem intakeSubsystem) {
        this.gamepad = gamepad;
        this.intakeSubsystem = intakeSubsystem;
        innerFlywheelRunning = false;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
//        intakeSubsystem.setOuterIntakeServoPower(1);
        gamepad.readButtons();
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            innerFlywheelRunning = !innerFlywheelRunning;
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            intakeSubsystem.setInnerIntakeServoPower(-1.0);
            intakeSubsystem.setOuterIntakeServoPower(-1.0);
        }
        else if (innerFlywheelRunning) {
            intakeSubsystem.setInnerIntakeServoPower(1.0);
            intakeSubsystem.setOuterIntakeServoPower(0);
        } else {
            intakeSubsystem.setInnerIntakeServoPower(0);
            intakeSubsystem.setOuterIntakeServoPower(1.01);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setOuterIntakeServoPower(0);
        intakeSubsystem.setInnerIntakeServoPower(0);
    }
}
