package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;
    private String centric = "field centric";
    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            if (centric == "field centric")
                centric = "robot centric";
            else
                centric = "field centric";
        }
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double rotation = -gamepad.getRightX();
        if (centric == "robot centric")
            drive.MecanumRobotCentricKitbot(x, y, rotation);
        else
            drive.MecanumDriveKitBot(x, y, rotation);
        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }
    }

    @Override
    public void end(boolean interrupted) { drive.MecanumDriveKitBot(0, 0, 0); }
}
