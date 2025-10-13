package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class FlyWheelSubsystem extends SubsystemBase {
    private final DcMotor motorizedFlywheel;
    private final CRServo rightFlyWheel, leftFlyWheel;
    private final JoinedTelemetry telemetry;

    public FlyWheelSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
        motorizedFlywheel = hardwareMap.get(DcMotor.class, "motorizedFlywheel");
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        this.telemetry = telemetry;
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        motorizedFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void flyWheelLaunch() {
        motorizedFlywheel.setPower(-0.85);
        telemetry.addData("Flywheel Power", motorizedFlywheel.getPower());
    }
    public void feed() {
        rightFlyWheel.setPower(1.0);
        leftFlyWheel.setPower(1.0);
        telemetry.addData("Right Flywheel Power", rightFlyWheel.getPower());
        telemetry.addData("Left Flywheel Power", leftFlyWheel.getPower());
    }

    public void reset() {
        motorizedFlywheel.setPower(0);
    }

    public void resetFeed() {
        rightFlyWheel.setPower(0);
        leftFlyWheel.setPower(0);
    }
}
