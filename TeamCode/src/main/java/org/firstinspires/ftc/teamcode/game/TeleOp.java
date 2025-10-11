package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")
public class TeleOp extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private PivotSubsystem pivotSubsystem;
    private GamepadEx driver;
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();


    @Override
    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, telemetryPacket, dashboard);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry, telemetryPacket);

        driver = new GamepadEx(gamepad1);

        mecanumDriveSubsystem.setDefaultCommand(
                new DriveCommand(driver, mecanumDriveSubsystem)
        );

        pivotSubsystem.setDefaultCommand(
                new PivotCommand(pivotSubsystem, limelightSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new LLAlignCommand(driver, mecanumDriveSubsystem, limelightSubsystem)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new FlywheelCommand(driver, flywheelSubsystem)
                )
        );
    }

    @Override
    public void init_loop() {}

    @Override
    public void loop() {
        //FTC Dashboard
        CommandScheduler.getInstance().run();
        dashboard.sendTelemetryPacket(telemetryPacket);
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
