package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")
public class TeleOp extends OpMode {
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    private final TelemetryPacket telemetryPacket = new TelemetryPacket();
    private final JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private GamepadEx driver;

    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, imu, joinedTelemetry);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, imu, telemetry, telemetryPacket, dashboard);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry, telemetryPacket);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, telemetryPacket);

        driver = new GamepadEx(gamepad1); // All keybindings are in readme

        mecanumDriveSubsystem.setDefaultCommand(
                new DriveCommand(driver, mecanumDriveSubsystem)
        );

        pivotSubsystem.setDefaultCommand(
                new PivotCommand(driver, pivotSubsystem, limelightSubsystem)
        );

        intakeSubsystem.setDefaultCommand(
                new IntakeCommand(intakeSubsystem)
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

        pivotSubsystem.resetPivotEncoder();
    }

    @Override
    public void init_loop() {}

    @Override
    public void loop() {
        // FTC Dashboard
        CommandScheduler.getInstance().run();
//        dashboard.sendTelemetryPacket(telemetryPacket);
//        telemetry.update();
        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}