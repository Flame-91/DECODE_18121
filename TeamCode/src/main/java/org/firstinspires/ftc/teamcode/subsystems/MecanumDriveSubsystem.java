package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveSubsystem extends SubsystemBase {
    IMU imu;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private final TelemetryPacket telemetryPacket;
    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        this.imu = imu;
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
    }

    @SuppressLint("DefaultLocale")
    public void drive(double x, double y, double rotation) {
        double headingRadians = -imu.getRobotYawPitchRollAngles().getYaw() * Math.PI / 180.0;

        double rotatedX = x * Math.cos(headingRadians) - y * Math.sin(headingRadians);
        double rotatedY = x * Math.sin(headingRadians) + y * Math.cos(headingRadians);

        double frontLeftPower = rotatedY + rotatedX + rotation;
        double frontRightPower = rotatedY - rotatedX - rotation;
        double backLeftPower = rotatedY - rotatedX + rotation;
        double backRightPower = rotatedY + rotatedX - rotation;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        String dataString = String.format("%.2f, %.2f, %.2f, %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("FL, FR, BL, BR", dataString);
        telemetryPacket.put("FL, FR, BL, BR", dataString);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
