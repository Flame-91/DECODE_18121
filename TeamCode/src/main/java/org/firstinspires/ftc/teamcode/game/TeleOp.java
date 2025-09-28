package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {
    private MecanumDriveSubsystem MacanumDriveSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private LLAlignCommand LLAlignCommand;
    private String team = "";
    LimelightSubsystem ll;

    public void init() {
        MacanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(gamepad1, MacanumDriveSubsystem, telemetry);
        CommandScheduler.getInstance().schedule(driveCommand);
        ll = new LimelightSubsystem(hardwareMap);
    }

    public void init_loop() {
        if (gamepad1.right_bumper) team = "red";
        if (gamepad1.left_bumper) team = "blue";
        telemetry.addData("team", team);
    }

    public void start() {}
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.a) {
            LLAlignCommand = new LLAlignCommand(MacanumDriveSubsystem, ll, gamepad1, telemetry);
            CommandScheduler.getInstance().schedule(LLAlignCommand);
//        } else if (LLAlignCommand != null) {
//            CommandScheduler.getInstance().cancel(LLAlignCommand); // driver has to hold button to keep running the command
        }

        dashboard.sendTelemetryPacket(packet);

        CommandScheduler.getInstance().run();
    }

    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}