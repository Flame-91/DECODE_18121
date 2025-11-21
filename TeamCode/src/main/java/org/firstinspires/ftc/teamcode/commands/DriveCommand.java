package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;
    private enum Centric {ROBOT, FIELD}
    private double speed = 1;
    private Centric centric = Centric.ROBOT;
    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            switch (centric) {
                case ROBOT:
                    centric = Centric.FIELD;
                    break;
                case FIELD:
                    centric = Centric.ROBOT;
                    break;
            }
        }
        double x = -gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double rotation = -gamepad.getRightX();
        switch (centric) {
            case ROBOT:
                drive.MecanumRobotCentricKitbot(x, y, rotation, speed);
                break;
            case FIELD:
                drive.MecanumDriveKitBot(x, y, rotation, speed);
                break;
        }

        if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            speed = 0.25;
        } else {
            speed = 1;
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }
    }

    @Override
    public void end(boolean interrupted) { drive.MecanumDriveKitBot(0, 0, 0, 67); }
}
