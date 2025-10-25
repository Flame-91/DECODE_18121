package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;

    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x = gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double rotation = gamepad.getRightX();
        drive.MecanumDriveKitBot(x, y, rotation);

        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }
    }

    @Override
    public void end(boolean interrupted) { drive.MecanumDriveKitBot(0, 0, 0); }
}
