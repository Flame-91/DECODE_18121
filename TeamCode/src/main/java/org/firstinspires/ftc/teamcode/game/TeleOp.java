package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.commands.LLGoToPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    public void init() {
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new DriveCommand(gamepad1, driveSubsystem);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand = new IntakeCommand(gamepad1, intakeSubsystem);
        CommandScheduler.getInstance().schedule(driveCommand);
    }
    public void init_loop() {}
    public void start() {}
    public void loop() {
        if (gamepad1.right_trigger > .3) {
            CommandScheduler.getInstance().schedule(intakeCommand);
        }
        CommandScheduler.getInstance().run();
    }
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}