package org.firstinspires.ftc.teamcode.ishaanCode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "FieldCentricDriveIshaan")
//adb connect 192.168.43.1:5555
public class FieldCentricDriveTeleop extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    public void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void initializeIMU() {
        IMU.Parameters IMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(IMUparameters);
        this.imu = imu;
    }
    public void fieldCentricDrive() {

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLateral = axial * Math.sin(heading) + lateral * Math.cos(heading);
        double adjustedAxial = -axial * Math.cos(heading) + lateral * Math.sin(heading);

        double max = Math.max(Math.max(Math.max(Math.abs(adjustedAxial), Math.abs(adjustedLateral)), Math.abs(yaw)), 1);

        double frontLeftPower = adjustedAxial + adjustedLateral + yaw;
        double frontRightPower = adjustedAxial - adjustedLateral - yaw;
        double backLeftPower = adjustedAxial - adjustedLateral + yaw;
        double backRightPower = adjustedAxial + adjustedLateral - yaw;

        frontLeft.setPower(frontLeftPower/max);
        frontRight.setPower(frontRightPower/max);
        backLeft.setPower(backLeftPower/max);
        backRight.setPower(backRightPower/max);
    }
    @Override
    public void init() {
        initializeMotors();
        initializeIMU();
    }
    @Override
    public void loop() {
        fieldCentricDrive();
    }
}