package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private String team = "";
    MecanumDriveSubsystem mecanumDriveSubsystem;
    LimelightSubsystem limelightSubsystem;
    FlywheelSubsystem flywheelSubsystem;

    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap);
    }

    public void init_loop() {
        if (gamepad1.right_bumper) team = "red";
        if (gamepad1.left_bumper) team = "blue";
        telemetry.addData("team", team);
    }

//    public void start() {}
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a) { // can't run LLAlign & DriveCommand at once since they both use MecanumDrive Subsystem
            CommandScheduler.getInstance().schedule(new LLAlignCommand(mecanumDriveSubsystem, limelightSubsystem, gamepad1, telemetry, dashboardTelemetry, packet, dashboard));
        } else {
            CommandScheduler.getInstance().schedule(new DriveCommand(gamepad1, mecanumDriveSubsystem, telemetry, dashboardTelemetry));
            if (gamepad1.y) CommandScheduler.getInstance().schedule(new FlywheelCommand(gamepad1, flywheelSubsystem, telemetry));
        }

        dashboard.sendTelemetryPacket(packet);

        CommandScheduler.getInstance().run();
    }

    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}