package org.firstinspires.ftc.teamcode.subsystems;

//Bad import? import static java.lang.Thread.sleep;

//Good import? import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlyWheelSubsystemOG extends SubsystemBase {
    private final DcMotor motorizedFlywheel;
    private final CRServo rightFlyWheel, leftFlyWheel;
    private final Gamepad gamepad;
    private boolean firstButtonPress;
    private boolean activateServos;
    private boolean hasReset;
    private final Telemetry telemetry;
    private final ElapsedTime elapsedTime;

    public FlyWheelSubsystemOG(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        elapsedTime = new ElapsedTime();
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firstButtonPress = true;
        activateServos = false;
        hasReset = false;
    }

    public void FlyWheelLaunch() {
        if (gamepad.a) {
            motorizedFlywheel.setPower(-1.0);
            if (firstButtonPress) {
                elapsedTime.reset();
                hasReset = true;
            }
            if (hasReset) {
                if (elapsedTime.seconds() > 1) {
                    activateServos = true;
                }
            }
            if (activateServos) {
                rightFlyWheel.setPower(-1.0);
                leftFlyWheel.setPower(1.0);
            }
            firstButtonPress = false;
        } else if (gamepad.x && !gamepad.a) {
            firstButtonPress = true;
            hasReset = false;
            motorizedFlywheel.setPower(.5);
            rightFlyWheel.setPower(1.0);
            leftFlyWheel.setPower(-1.0);
        } else if (gamepad.x) {
            motorizedFlywheel.setPower(.5);
            rightFlyWheel.setPower(1.0);
            leftFlyWheel.setPower(-1.0);
        } else {
            firstButtonPress = true;
            hasReset = false;
            motorizedFlywheel.setPower(0);
            rightFlyWheel.setPower(0);
            leftFlyWheel.setPower(0);
        }
        telemetry.addData("Motor speed", motorizedFlywheel.getPower());
        telemetry.addData("Elapsed time (seconds)", elapsedTime.seconds());
        telemetry.addData("Right servo power", rightFlyWheel.getPower());
        telemetry.addData("Left servo power", leftFlyWheel.getPower());
        telemetry.addData("firstButtonPress", firstButtonPress);
        telemetry.addData("hasReset", hasReset);
        telemetry.addData("activateServos", activateServos);
        telemetry.update();
    }
}