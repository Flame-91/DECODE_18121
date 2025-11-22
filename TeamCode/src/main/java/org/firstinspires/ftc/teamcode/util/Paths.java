package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain preload_score_red;
    public PathChain to_reload_red;
    public PathChain to_reload_red_1;
    public PathChain to_reload_red_2;
    public PathChain to_score_red;
    public PathChain preload_score_blue_1;
    public PathChain preload_score_blue_2;
    public PathChain to_reload_blue_1;
    public PathChain to_reload_blue_2;
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

        to_reload_red = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.000, 124.000), new Pose(27.083, 27.743))
                )
                .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(45))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_reload_red_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.083, 27.743), new Pose(20.477, 20.807))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_reload_red_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.477, 20.807), new Pose(15.193, 15.358))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
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


        preload_score_blue_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(42.141, 103.211))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(320))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
        preload_score_blue_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.141, 103.211), new Pose(22.000, 124.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(320))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
        to_reload_blue_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 124.000), new Pose(46.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(0))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_reload_blue_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(46.000, 84.000), new Pose(24.338, 83.831))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        to_score_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24.338, 83.831), new Pose(21.634, 123.493))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(320))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
    }
}
