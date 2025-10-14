package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.GlobalConstants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "game")
public class Autonomous extends OpMode {
    private IMU imu;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
    }

    @Override
    public void loop() {
        // auto
    }

    @Override
    public void stop() {
        GlobalConstants.imuOffset = imu.getRobotYawPitchRollAngles().getYaw();
    }
}
