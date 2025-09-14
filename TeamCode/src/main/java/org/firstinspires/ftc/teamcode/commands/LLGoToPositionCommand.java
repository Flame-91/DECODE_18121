package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.LinearController;

public class LLGoToPositionCommand extends CommandBase {
    LimelightSubsystem ll;
    final private double xTarget;
    final private double yTarget;
    private final MecanumDriveSubsystem drive;
//    private final Gamepad gamepad;
    LinearController xController = new LinearController((1.0/3.6), 0, -0.5, 0.5);
    LinearController yController = new LinearController((1.0/3.6), 0, -0.5, 0.5);
    LinearController yawController = new LinearController((1.0/250), 0, -0.5, 0.5);
    //    final private double
    public LLGoToPositionCommand(MecanumDriveSubsystem drive, double xTarget, double yTarget) {
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.drive = drive;
//        this.gamepad = gamepad;
    }

    @Override
    public void execute() {//(double xTarget in meters, double yTarget, double xRobot, double yRobot, double heading, double yawError) {
        // Position error in field space
        if (ll.hasTarget()) {
            double xRobot = ll.getBotPose()[0];
            double yRobot = ll.getBotPose()[1];
            double heading = ll.getBotPose()[5];

            double yawError = ll.getYaw();


            double errorX = xTarget - xRobot;
            double errorY = yTarget - yRobot;

            // Linear controllers (you already have)
            double vx = xController.calculate(errorX);
            double vy = yController.calculate(errorY);

            // Rotate into robot space
            double xCmd = vx * Math.cos(heading) + vy * Math.sin(heading);
            double yCmd = -vx * Math.sin(heading) + vy * Math.cos(heading);

            // Rotation correction
            double rx = yawController.calculate(yawError);

            // Mecanum drive
            double fl = yCmd + xCmd + rx;
            double fr = yCmd - xCmd - rx;
            double bl = yCmd - xCmd + rx;
            double br = yCmd + xCmd - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;

            drive.drive(fl, fr, bl, br);
        }
    }


}
