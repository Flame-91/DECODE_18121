package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.GlobalConstants;

public class MecanumDriveSubsystem extends SubsystemBase {
    IMU imu;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public MecanumDriveSubsystem(HardwareMap hardwareMap, IMU imu, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu = imu;
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("backLeftMotorPower", getBackLeftMotorPower());
        telemetry.addData("backRightMotorPower", getBackRightMotorPower());
        telemetry.addData("frontLeftMotorPower", getFrontLeftMotorPower());
        telemetry.addData("frontRightMotorPower", getFrontRightMotorPower());

        telemetryPacket.put("backLeftMotorPower", getBackLeftMotorPower());
        telemetryPacket.put("backRightMotorPower", getBackRightMotorPower());
        telemetryPacket.put("frontLeftMotorPower", getFrontLeftMotorPower());
        telemetryPacket.put("frontRightMotorPower", getFrontRightMotorPower());
    }

    public void drive(double x, double y, double rotation) {
        double headingRadians = -(imu.getRobotYawPitchRollAngles().getYaw()) * Math.PI / 180.0;

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
    }

    public void robotCentricDrive(double x, double y, double rotation) {
        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

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

    public double getBackLeftMotorPower() { return backLeft.getPower(); }
    public double getBackRightMotorPower() { return backRight.getPower(); }
    public double getFrontLeftMotorPower() { return frontLeft.getPower(); }
    public double getFrontRightMotorPower() { return frontRight.getPower(); }
}
