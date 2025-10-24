package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotor backLeft, backRight, frontLeft, frontRight;
    private final Telemetry telemetry;
//    private final IMU imu;
    public MecanumDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        this.telemetry = telemetry;
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters imuParams = new IMU.Parameters(
//                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        imu.initialize(imuParams);
//        this.imu = imu;
        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("frontLeftPower", getFrontLeftPower());
        telemetry.addData("frontRightPower", getFrontRightPower());
        telemetry.addData("backLeftPower", getBackLeftPower());
        telemetry.addData("backRightPower", getBackRightPower());
    }
    public void MecanumDriveKitBot(double x, double y, double rotation) {
//        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        y = x * Math.cos(heading) - y * Math.sin(heading);
//        x = x * Math.sin(heading) + y * Math.cos(heading);

        double frontLeftPower = (y + x + rotation);
        double backLeftPower = (y - x + rotation);
        double frontRightPower = (y - x - rotation);
        double backRightPower = (y + x - rotation);

        double max = Math.max(1, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }
}