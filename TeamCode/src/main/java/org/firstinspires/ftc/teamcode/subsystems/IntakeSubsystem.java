package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo outerIntakeServo, innerIntakeServo;
    private final Telemetry telemetry;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        outerIntakeServo = hardwareMap.get(CRServo.class, "outerIntakeServo");
        innerIntakeServo = hardwareMap.get(CRServo.class, "innerIntakeServo");
        this.telemetry = telemetry;
        outerIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        telemetry.addData("innerIntakeServoPower", getIntakeServoPower());
        telemetry.update();
    }

    public void setIntakeServoPower(double power) {
        outerIntakeServo.setPower(power);
        innerIntakeServo.setPower(power);
    }

    public double getIntakeServoPower() {
        return outerIntakeServo.getPower();
    }
}
