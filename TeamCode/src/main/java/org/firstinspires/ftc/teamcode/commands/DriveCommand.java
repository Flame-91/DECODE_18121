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

    public void execute() {
        if (!gamepad.a) {
            double x = -gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double rotation = gamepad.right_stick_x;
            drive.drive(x, y, rotation);
        } else {
//            double correctX;
//            double correctY;
            double buffer = 0.03;
            double xCorrection = 0;
            double yCorrection = 0;
            double yawCorrection = 0;

            if (ll.distanceFromTag()[0] > buffer) {
                xCorrection = -0.2;
            } else if (ll.distanceFromTag()[0] < -buffer) {
                xCorrection = 0.2;
            }

            if (ll.distanceFromTag()[1] > buffer) {
                yCorrection = -0.2;
            } else if (ll.distanceFromTag()[1] < -buffer) {
                yCorrection = 0.2;
            }

            if (ll.getYaw() > 0) {
                yawCorrection = -0.15;
            } else {
                yawCorrection = 0.15;
            }

            drive.drive(-xCorrection, -yCorrection, yawCorrection);
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
