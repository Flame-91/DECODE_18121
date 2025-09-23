package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private LLAlignCommand LLAlignCommand;
    private String team = "";

    public void init() {
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new DriveCommand(gamepad1, driveSubsystem);
        CommandScheduler.getInstance().schedule(driveCommand);

        if (team.isEmpty()) {
            if (gamepad1.right_bumper) team = "red";
            if (gamepad1.left_bumper) team = "blue";
        }
    }
    public void init_loop() {}
    public void start() {}
    public void loop() {
        if (gamepad1.a) {
            LLAlignCommand = new LLAlignCommand(driveSubsystem);
            CommandScheduler.getInstance().schedule(LLAlignCommand);
        } else {
            CommandScheduler.getInstance().cancel(LLAlignCommand); // driver has to hold button to keep running the command
        }

        CommandScheduler.getInstance().run();
    }
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}