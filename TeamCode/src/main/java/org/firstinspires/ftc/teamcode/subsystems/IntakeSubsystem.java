package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("intakeServoPower", getIntakeServoPower());

        telemetryPacket.put("intakeServoPower", getIntakeServoPower());
    }

    public void setIntakeServoPower(double power) { intakeServo.setPower(power); }
    public double getIntakeServoPower() { return intakeServo.getPower(); }
}
