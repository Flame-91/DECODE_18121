package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("intakeServoPower", getIntakeServoPower());

        telemetryPacket.put("intakeServoPower", getIntakeServoPower());
    }

    public void setIntakeServoPower(double power) {
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }
    public double getIntakeServoPower() {
        return leftIntakeServo.getPower();
    }
}