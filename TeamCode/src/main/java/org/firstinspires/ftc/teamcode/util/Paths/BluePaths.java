package org.firstinspires.ftc.teamcode.util.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePaths {
    public PathChain shoot1;
    public PathChain reload11;
    public PathChain reload12;
    public PathChain shoot2;
    public PathChain reload21;
    public PathChain reload22;
    public PathChain score3;
    public PathChain gate;
    public PathChain reload31;
    public PathChain reload32;
    public PathChain score4;

    public BluePaths(Follower follower) {
        shoot1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(54.800, 18.600))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                .build();

        reload11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.800, 18.600), new Pose(43.500, 83.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(0))
                .build();

        reload12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.500, 83.700), new Pose(22.200, 83.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.200, 83.800), new Pose(44.900, 98.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                .build();

        reload21 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.900, 98.500), new Pose(40.400, 59.800))
                )
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                .build();

        reload22 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.400, 59.800), new Pose(22.500, 60.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.500, 60.300), new Pose(53.400, 90.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))
                .build();

        gate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(53.400, 90.700), new Pose(15.400, 65.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(270))
                .build();

        reload31 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.400, 65.400), new Pose(18.300, 59.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                .build();

        reload32 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.300, 59.500), new Pose(11.900, 50.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        score4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(11.900, 50.100), new Pose(51.053, 92.947))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))
                .build();
    }
}
