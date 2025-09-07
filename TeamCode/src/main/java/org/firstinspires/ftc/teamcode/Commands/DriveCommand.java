package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    MecanumDriveSubsystem drive;
    Gamepad gamepad;
    public DriveCommand(Gamepad gamepad, MecanumDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }
    public void initialize() {}

    public void execute() {
        double x = -gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;
        drive.drive(x, y, rotation);
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0);
    }
}
