package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommandNew extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;
    private String centric;
    private final LimelightSubsystem ll;
    String team;
    private final Pose blue_base;
    private final Pose red_base;
    private double[] pedroCoordinates;
    private boolean toBase = false;
    Pose beforeBase;
    private double heading;
    Follower follower;

    public DriveCommandNew(GamepadEx gamepad, MecanumDriveSubsystem drive, Follower follower, LimelightSubsystem ll) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.follower = follower;
        this.ll = ll;
        blue_base = new Pose(105.331136738056, 33.21252059308073, Math.toRadians(90)); //Is it degrees???
        red_base = new Pose(38.66886326194399, 33.44975288303131, Math.toRadians(90)); //Is it degrees???
        pedroCoordinates = new double[5];
        beforeBase = new Pose(1, 1, 1); //to get rid of warning
        heading = 0.0;
        centric = "robot";
        team = "unselected";
        addRequirements(drive);
    }

    @Override
    public void execute() {
        gamepad.readButtons();
        if (toBase) {
            if (team.equals("blue")) {
                PathChain bluePath = follower
                        .pathBuilder()
                        .addPath(new BezierLine(beforeBase, blue_base))
                        .setLinearHeadingInterpolation(heading, 90)
                        .setTranslationalConstraint(1.5)
                        .setHeadingConstraint(Math.toRadians(5))
                        .setVelocityConstraint(2)
                        .setTValueConstraint(0.98)
                        .setTimeoutConstraint(300)
                        .build();
                follower.followPath(bluePath);
                if (!follower.isBusy()) {
                    toBase = false;
                }
            } else if (team.equals("red")) {
                PathChain redPath = follower
                        .pathBuilder()
                        .addPath(new BezierLine(beforeBase, red_base))
                        .setLinearHeadingInterpolation(heading, 90)
                        .setTranslationalConstraint(1.5)
                        .setHeadingConstraint(Math.toRadians(5))
                        .setVelocityConstraint(2)
                        .setTValueConstraint(0.98)
                        .setTimeoutConstraint(300)
                        .build();
                follower.followPath(redPath);
                if (!follower.isBusy()) {
                    toBase = false;
                }
            } else if (team.equals("unselected")) {

                toBase = false;
            }
        } else {
            double x = gamepad.getLeftX();
            double y = -gamepad.getLeftY();
            double rotation = gamepad.getRightX();
            if (centric.equals("field")) {
                drive.drive(x, y, rotation);
            } else if (centric.equals("robot")) {
                drive.robotCentricDrive(x, y, rotation);
            } else {
                drive.robotCentricDrive(x, y, rotation); //Backup
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if (centric.equals("robot")) {
                centric = "field";
            } else if (centric.equals("field")) {
                centric = "robot";
            }
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            beforeBase = follower.getPose();
            heading = follower.getHeading();
            toBase = true;
        }
    }

    public void changeTeam(String team) {
        this.team = team;
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
