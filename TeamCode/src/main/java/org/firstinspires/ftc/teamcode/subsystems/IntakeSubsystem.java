package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        this.telemetry = telemetry;
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        telemetry.addData("intakeServoPower", getIntakeServoPower());
        telemetry.update();
    }

    public void setIntakeServoPower(double power) {
        intakeServo.setPower(power);
    }

    public double getIntakeServoPower() {
        return intakeServo.getPower();
    }
}
