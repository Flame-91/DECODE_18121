package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")
public class TeleOp extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private String team = "";

    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private FlywheelSubsystem flywheelSubsystem;

    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry, dashboard);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, dashboard);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, dashboard);
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) team = "red";
        if (gamepad1.left_bumper) team = "blue";
        telemetry.addData("team", team);
    }

    @Override
    public void start() {
        mecanumDriveSubsystem.setDefaultCommand(
                new DriveCommand(gamepad1, mecanumDriveSubsystem)
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        boolean aPressed = gamepad1.a;
        boolean yPressed = gamepad1.y;

        if (aPressed && !lastA) {
            CommandScheduler.getInstance().schedule(
                    new LLAlignCommand(gamepad1, mecanumDriveSubsystem, limelightSubsystem)
            );
        }

        if (yPressed && !lastY) {
            CommandScheduler.getInstance().schedule(
                    new FlywheelCommand(gamepad1, flywheelSubsystem)
            );
        }

        lastA = aPressed;
        lastY = yPressed;

        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}
