package org.firstinspires.ftc.teamcode.util.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedPaths {
    public PathChain score_1;
    public PathChain reload_1_1;
    public PathChain reload_1_2;
    public PathChain score_2;
    public PathChain reload_2_1;
    public PathChain reload_2_2;
    public PathChain score_3;
    public PathChain gate;
    public PathChain reload_3_1;
    public PathChain reload_3_2;
    public PathChain score_4;

    public RedPaths(Follower follower) {
        score_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.250, 8.000), new Pose(87.991, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_1_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.991, 18.000), new Pose(103.414, 83.932))
                )
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_1_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.414, 83.932), new Pose(126.467, 84.095))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        score_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.467, 84.095), new Pose(94.485, 93.835))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_2_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(94.485, 93.835), new Pose(102.277, 58.769))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_2_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.277, 58.769), new Pose(127.116, 59.094))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        score_3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.116, 59.094), new Pose(86.205, 86.205))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(46))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        gate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.205, 86.205), new Pose(127.928, 70.458))
                )
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(90))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_3_1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.928, 70.458), new Pose(128.735, 49.527))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        reload_3_2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128.735, 49.527), new Pose(133.993, 47.661))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        score_4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.587, 43.761), new Pose(86.205, 86.205))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(46))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();
    }
}
