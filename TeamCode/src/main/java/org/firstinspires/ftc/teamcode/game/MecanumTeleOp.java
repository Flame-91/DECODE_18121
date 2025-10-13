package org.firstinspires.ftc.teamcode.game;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlyWheelCommand;
import org.firstinspires.ftc.teamcode.commands.ServoCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Mecanum TeleOp")
public class MecanumTeleOp extends OpMode {
    private final JoinedTelemetry joinedTelemetry =
            new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    MecanumDriveSubsystem driveSubsystem;
    FlyWheelSubsystem flyWheelSubsystem;
    DriveCommand driveCommand;
    FlyWheelCommand flyWheelCommand;
    ServoCommand servoCommand;
    boolean pressedA = false;
    boolean pressedX = false;

    private GamepadEx driver;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);

        driveSubsystem = new MecanumDriveSubsystem(hardwareMap, joinedTelemetry);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, joinedTelemetry);

        driveCommand = new DriveCommand(driver, driveSubsystem);
        flyWheelCommand = new FlyWheelCommand(gamepad1, flyWheelSubsystem);
        servoCommand = new ServoCommand(gamepad1, flyWheelSubsystem);

        driveSubsystem.setDefaultCommand(driveCommand);
        joinedTelemetry.addData("Status", "Running");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x && !pressedX) {
            CommandScheduler.getInstance().schedule(servoCommand);
            pressedX = true;
        } else {
            pressedX = false;
        }

        if (gamepad1.a && !pressedA) {
            CommandScheduler.getInstance().schedule(flyWheelCommand);
            pressedA = true;
        } else {
            pressedA = false;
        }

        CommandScheduler.getInstance().run();
        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        joinedTelemetry.addData("Status", "Stopped");
        joinedTelemetry.update();
    }
}
