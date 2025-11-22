package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain preload_score_red;
    public PathChain to_reload_red;
    public PathChain to_score_red;
    public PathChain preload_score_blue;
    public PathChain to_reload_blue;
    public PathChain to_score_blue;
    public Paths(Follower follower) {
        preload_score_red = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88,8), new Pose(119,123)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(220))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_reload_red = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119,123), new Pose(16,16)))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(40))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_score_red = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16,16), new Pose(119,123)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(220))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
        preload_score_blue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56,8), new Pose(24,126)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(320))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
        to_reload_blue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24,126), new Pose(128,16)))
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(130))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
        to_score_blue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128,16), new Pose(24,126)))
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(320))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
    }
}
