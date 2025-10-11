package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheelSubsystem {
    private final DcMotor motorizedFlywheel;
    private final CRServo rightFlyWheel, leftFlyWheel;
    private final Gamepad gamepad;
    public FlyWheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        this.gamepad = gamepad;
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void FlyWheelLaunch() {
        if (gamepad.a) {
            motorizedFlywheel.setPower(1.0);
            rightFlyWheel.setPower(-1.0);
            leftFlyWheel.setPower(1.0);
        }
        else {
            motorizedFlywheel.setPower(0);
            rightFlyWheel.setPower(0);
            leftFlyWheel.setPower(0);
        }
    }
}