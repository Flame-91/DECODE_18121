package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain preload_score;
    public PathChain to_reload;
    public PathChain to_score;

    public Paths(Follower follower) {

        preload_score = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(61,11), new Pose(119,123)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(220))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_reload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119,123), new Pose(16,16)))
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(40))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_score = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16,16), new Pose(119,123)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(220))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
    }
}
