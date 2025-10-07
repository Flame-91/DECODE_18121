package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotor flywheelMotor;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
    }

    @Override
    public void periodic() {
        telemetry.addData("flywheelPower", getFlywheelMotorPower());
        telemetry.addData("servoPower", getServosPower());

        telemetryPacket.put("flywheelPower", getFlywheelMotorPower());
        telemetryPacket.put("servoPower", getServosPower());
    }

    private double clamp(double power) {
        if (power < -1) {
            return -1;
        } else if (power > 1) {
            return 1;
        } else {
            return power;
        }
    }

    public void setFlywheelMotorPower(double power) {
        flywheelMotor.setPower(clamp(power));
    }

    public void setServosPower(double power) {
        rightServo.setPower(clamp(power));
        leftServo.setPower(clamp(power));
    }

    public double getFlywheelMotorPower() { return flywheelMotor.getPower(); }
    public double getServosPower() { return rightServo.getPower(); }
}
