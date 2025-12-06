package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx driver;
    private double[] pedroCoordinates;
    private boolean knowPose;
    private DriveCommand driveCommand;
    private Follower follower;
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot (
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, imu, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, imu, telemetry, telemetryPacket, dashboard);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry, telemetryPacket);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, telemetryPacket);

        follower = Constants.createFollower(hardwareMap);

        if (limelightSubsystem.getBotPosePose3D() == null) {
            pedroCoordinates = new double[5];
            knowPose = false;
        } else {
            Pose3D startingPose3d = limelightSubsystem.getBotPosePose3D();
            pedroCoordinates = new double[]{
                    (39.3701 * (startingPose3d.getPosition().x)) + 72,
                    (39.3701 * (startingPose3d.getPosition().y)) + 72,
                    Math.toRadians(startingPose3d.getOrientation().getYaw()),
            };
            follower.setStartingPose(new Pose(pedroCoordinates[0], pedroCoordinates[1], pedroCoordinates[2]));
            knowPose = true;
        }

        driver = new GamepadEx(gamepad1); // All keybindings are in readme
        driveCommand = new DriveCommand(driver, mecanumDriveSubsystem, follower, knowPose);

        mecanumDriveSubsystem.setDefaultCommand(
                driveCommand
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

        flywheelSubsystem.setDefaultCommand(
                new FlywheelCommand(driver, flywheelSubsystem)
        );

        pivotSubsystem.resetPivotEncoder();

    }

    @Override
    public void init_loop() {
        driver.readButtons();
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            driveCommand.changeTeam("red");
        } else if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            driveCommand.changeTeam("blue");
        }
    }

    @Override
    public void loop() {
        if (limelightSubsystem.getBotPosePose3D() != null && knowPose) {
            Pose3D botPose = limelightSubsystem.getBotPosePose3D();
            pedroCoordinates[0] = (39.3701*(botPose.getPosition().x)) + 72;
            pedroCoordinates[1] = (39.3701*(botPose.getPosition().y)) + 72;
            pedroCoordinates[2] = Math.toRadians(botPose.getOrientation().getYaw());
            pedroCoordinates[3] = Math.toRadians(botPose.getOrientation().getPitch());
            pedroCoordinates[4] = Math.toRadians(botPose.getOrientation().getRoll());
            follower.setPose(new Pose(pedroCoordinates[0], pedroCoordinates[1], pedroCoordinates[2]));
            driveCommand.updateFollower(follower.getPose()); //add more methods here to update individual command's followers, not sure if necessary but just in case
        } else if (limelightSubsystem.getBotPosePose3D() != null && !knowPose) {
            Pose3D botPose = limelightSubsystem.getBotPosePose3D();
            pedroCoordinates[0] = (39.3701*(botPose.getPosition().x)) + 72;
            pedroCoordinates[1] = (39.3701*(botPose.getPosition().y)) + 72;
            pedroCoordinates[2] = Math.toRadians(botPose.getOrientation().getYaw());
            pedroCoordinates[3] = Math.toRadians(botPose.getOrientation().getPitch());
            pedroCoordinates[4] = Math.toRadians(botPose.getOrientation().getRoll());
            follower.setStartingPose(new Pose(pedroCoordinates[0], pedroCoordinates[1], pedroCoordinates[2]));
            driveCommand.setFollowerStartingPose(follower.getPose());
        }
        // FTC Dashboard
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