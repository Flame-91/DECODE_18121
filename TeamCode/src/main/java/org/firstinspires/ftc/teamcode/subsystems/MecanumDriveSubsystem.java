package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotor backLeft, backRight, frontLeft, frontRight;
    private final Telemetry telemetry;
    public final IMU imu;
//    private final IMU imu;
    public MecanumDriveSubsystem(HardwareMap hardwareMap, IMU imu, Telemetry telemetry) {
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
        this.imu = imu;

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
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower = (rotatedX + rotatedY + rotation);
        double backLeftPower = (rotatedY - rotatedX + rotation);
        double frontRightPower = (rotatedY - rotatedX - rotation);
        double backRightPower = (rotatedY + rotatedX - rotation);

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

    public void resetIMU() {
        imu.resetYaw();
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