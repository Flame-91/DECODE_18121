package org.firstinspires.ftc.teamcode.ishaanCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class FlyWheelSubsystem {
    private DcMotor motorizedFlywheel;
    public FlyWheelSubsystem(HardwareMap hardwareMap) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void FlyWheelLaunch() {
        if (gamepad1.a) {
            motorizedFlywheel.setPower(1.0);
        } else {
            motorizedFlywheel.setPower(0);
        }
    }
}
