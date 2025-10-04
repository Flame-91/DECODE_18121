package org.firstinspires.ftc.teamcode.commands;

import android.annotation.SuppressLint;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final Telemetry dashboardTelemetry;

    public DriveCommand(Gamepad gamepad, MecanumDriveSubsystem drive, Telemetry telemetry, Telemetry dashboardTelemetry) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.dashboardTelemetry = dashboardTelemetry;
        addRequirements(drive);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void execute() {
        double x = gamepad.left_stick_x;
        double y = gamepad.left_stick_y;
        double rotation = -gamepad.right_stick_x;
        telemetry.addData("Movement", String.format("x: %f, y: %f rotation: %f", x, y, rotation));
        dashboardTelemetry.addData("Movement", String.format("x: %f, y: %f rotation: %f", x, y, rotation));
        telemetry.update();
        dashboardTelemetry.update();
        drive.drive(x, y, rotation);
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
