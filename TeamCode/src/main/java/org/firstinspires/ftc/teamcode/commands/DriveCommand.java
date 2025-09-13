package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    MecanumDriveSubsystem drive;
    Gamepad gamepad;
    LimelightSubsystem ll;
    public DriveCommand(Gamepad gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    public void initialize() {}

    public String team() {
        if (gamepad.left_bumper) {
            return "blue";
        } else if (gamepad.right_bumper) {
            return "red";
        }
        return "";
    }

    public void execute() {
        if (gamepad.a && ll.hasTarget()) {
            double[] corrections = ll.autoAlign();

            double xCorrection = corrections[0];
            double yCorrection = corrections[1];
            double yawCorrection = corrections[2];

            drive.drive(-xCorrection, -yCorrection, yawCorrection);
        } else {
            double x = -gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double rotation = gamepad.right_stick_x;

            drive.drive(x, y, rotation);
        }
    }

//    public void execute(boolean auto, String team) {
//
//    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
