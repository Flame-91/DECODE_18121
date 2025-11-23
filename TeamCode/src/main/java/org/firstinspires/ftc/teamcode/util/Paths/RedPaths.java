package org.firstinspires.ftc.teamcode.util.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedPaths {
    public PathChain shoot1;
    public PathChain reload11;
    public PathChain reload12;
    public PathChain shoot2;
    public PathChain reload21;
    public PathChain reload22;
    public PathChain shoot3;
    public PathChain gate;
    public PathChain reload31;
    public PathChain shoot4;

    public RedPaths(Follower follower) {
        shoot1 = follower
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

        reload11 = follower
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

        reload12 = follower
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

        shoot2 = follower
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

        reload21 = follower
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

        reload22 = follower
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

        shoot3 = follower
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

        reload31 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.928, 70.458), new Pose(134.587, 43.761))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setTranslationalConstraint(1.5)
                .setHeadingConstraint(Math.toRadians(5))
                .setVelocityConstraint(2)
                .setTValueConstraint(0.98)
                .setTimeoutConstraint(300)
                .build();

        shoot4 = follower
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
