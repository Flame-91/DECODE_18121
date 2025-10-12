package org.firstinspires.ftc.teamcode.subsystems;

//import static java.lang.Thread.sleep;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.robotcore.util.ElapsedTime;

public class FlyWheelSubsystem extends SubsystemBase {
    private final DcMotor motorizedFlywheel;
    private final CRServo rightFlyWheel, leftFlyWheel;
    private final Gamepad gamepad;
    private boolean buttonAlreadyPressed;
    private final Telemetry telemetry;
//    private final ElapsedTime elapsedTime;

    public FlyWheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elapsedTime = new ElapsedTime();
        buttonAlreadyPressed = false;
    }

    public void FlyWheelLaunch() {
        if (gamepad.a) {
            motorizedFlywheel.setPower(-1.0);
//            if (!buttonAlreadyPressed) sleep(1000);
            rightFlyWheel.setPower(-1.0);
            leftFlyWheel.setPower(1.0);
            buttonAlreadyPressed = true;
        } //else if (gamepad.x) {
//            buttonAlreadyPressed = false;
//            motorizedFlywheel.setPower(1.0);
//            rightFlyWheel.setPower(1.0);
//            leftFlyWheel.setPower(-1.0);
        /*}*/ else {
            motorizedFlywheel.setPower(0);
            rightFlyWheel.setPower(0);
            leftFlyWheel.setPower(0);
        }

    }
}
//            rightFlyWheel.setPower(-1.0);
//            leftFlyWheel.setPower(1.0);