package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class DriveCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final GamepadEx gamepad;
    private String centric;
    private final LimelightSubsystem ll;
    String team;
    private final Pose blue_base;
    private final Pose red_base;
    Follower follower;

    public DriveCommand(GamepadEx gamepad, MecanumDriveSubsystem drive, Follower follower, LimelightSubsystem ll) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.follower = follower;
        this.ll = ll;
        blue_base = new Pose(105.331136738056, 33.21252059308073, Math.toRadians(90)); //Is it degrees???
        red_base = new Pose(38.66886326194399, 33.44975288303131, Math.toRadians(90)); //Is it degrees???
        addRequirements(drive);
        centric = "robot";
        team = "unselected";
    }

    @Override
    public void execute() {
        double x = gamepad.getLeftX();
        double y = -gamepad.getLeftY();
        double rotation = gamepad.getRightX();
        Pose3D botPose = ll.getBotPosePose3D();
        double[] pedroCoordinates = {
                (39.3701*(botPose.getPosition().x)) + 72,
                (39.3701*(botPose.getPosition().y)) + 72,
                Math.toRadians(botPose.getOrientation().getYaw()),
                Math.toRadians(botPose.getOrientation().getPitch()),
                Math.toRadians(botPose.getOrientation().getRoll())
        };
        gamepad.readButtons();
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (centric.equals("robot")) {
                centric = "field";
            } else if (centric.equals("field")) {
                centric = "robot";
            }
        }

        if (centric.equals("field")) {
            drive.drive(x, y, rotation);
        } else if (centric.equals("robot")) {
            drive.robotCentricDrive(x, y, rotation);
        } else {
            drive.robotCentricDrive(x, y, rotation); //Backup
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            drive.resetIMU();
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            follower.setStartingPose(new Pose(pedroCoordinates[0], pedroCoordinates[1], Math.toRadians(pedroCoordinates[2])));
            if (team.equals("blue")) {
                PathChain bluePath = follower
                        .pathBuilder();
            }
        }
    }

    public void changeTeam(String team) {
        this.team = team;
    }

    @Override
    public void end(boolean interrupted) { drive.drive(0, 0, 0); }
}
