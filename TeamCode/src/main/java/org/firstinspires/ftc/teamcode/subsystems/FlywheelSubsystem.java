package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotor flywheelMotor;
    private final Telemetry telemetry;
    private final FtcDashboard dashboard;
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetry;
        this.dashboard = dashboard;
    }

    public void setFlywheelMotor(double power) {
        if (power < -1 || power > 1) return;
        flywheelMotor.setPower(power);

        telemetry.addData("Flywheel Power", power);
        telemetryPacket.put("Flywheel Power", power);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    public void setServosPower(double power) {
        if (power < -1 || power > 1) return;
        rightServo.setPower(power);
        leftServo.setPower(power);

        telemetry.addData("Servo Power", power);
        telemetryPacket.put("Servo Power", power);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
