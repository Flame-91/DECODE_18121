package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    public DriveCommand(Gamepad gamepad, MecanumDriveSubsystem drive, Telemetry telemetry) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        double rotation = -gamepad.right_stick_x;
//        if (Math.abs(rotation) > 0) {
//            telemetry.addData("rightstick", "true");
//        }
        drive.drive(x, y, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
