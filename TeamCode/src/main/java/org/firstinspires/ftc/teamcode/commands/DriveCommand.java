package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;
    private String centric;
    String team;
    PathChain blue_base;
    PathChain red_base;
    Follower follower;

    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive, Follower follower) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
        centric = "robot";
        team = "unselected";
    }

    @Override
    public void execute() {
        double x = gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double rotation = gamepad.getRightX();
        gamepad.readButtons();
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (centric.equals("robot")) {
                centric = "field";
            } else if (centric.equals("field")) {
                centric = "robot";
            }
        }

        if (centric.equals("field")) {
            drive.drive(x, y, rotation);
        } else if (centric.equals("robot")) {
            drive.robotCentricDrive(x, y, rotation);
        } else {
            drive.robotCentricDrive(x, y, rotation); //Backup
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {

        }
    }

    public void changeTeam(String team) {
        this.team = team;
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
