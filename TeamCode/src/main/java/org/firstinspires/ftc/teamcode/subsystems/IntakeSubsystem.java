package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;
    private final JoinedTelemetry telemetry;
    public IntakeSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
        this.telemetry = telemetry;

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("intakeServoPower", getIntakeServoPower());
    }

    public void setIntakeServoPower(double power) { intakeServo.setPower(power); }
    public double getIntakeServoPower() { return intakeServo.getPower(); }
}
