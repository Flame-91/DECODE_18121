package org.firstinspires.ftc.teamcode.kitbot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheelSubsystem {
    private final DcMotor motorizedFlywheel;
    private final CRServo rightFlyWheel;
    private final CRServo leftFlyWheel;
    public FlyWheelSubsystem(HardwareMap hardwareMap) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void FlyWheelLaunch() {
        if (gamepad1.a) {
            motorizedFlywheel.setPower(1.0);
            rightFlyWheel.setPower(1.0);
            leftFlyWheel.setPower(1.0);
        }
        else {
            motorizedFlywheel.setPower(0);
            rightFlyWheel.setPower(0);
            leftFlyWheel.setPower(0);
        }
    }
}