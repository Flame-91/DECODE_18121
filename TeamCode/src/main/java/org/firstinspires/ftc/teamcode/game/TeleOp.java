package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private LLAlignCommand LLAlignCommand;
    private String team = "";
    LimelightSubsystem ll;

    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(gamepad1, mecanumDriveSubsystem, telemetry);
        CommandScheduler.getInstance().schedule(driveCommand);
        ll = new LimelightSubsystem(hardwareMap);
    }

    public void init_loop() {
        if (gamepad1.right_bumper) team = "red";
        if (gamepad1.left_bumper) team = "blue";
        telemetry.addData("team", team);
    }

//    public void start() {}
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a && (LLAlignCommand == null || !LLAlignCommand.isScheduled())) {
            LLAlignCommand = new LLAlignCommand(mecanumDriveSubsystem, ll, gamepad1, telemetry);
            CommandScheduler.getInstance().schedule(LLAlignCommand);
        }

        dashboard.sendTelemetryPacket(packet);

        CommandScheduler.getInstance().run();
    }

    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}