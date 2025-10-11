package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {
    private final DcMotor pivot;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;

    public PivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        pivot = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
    }

    @Override
    public void periodic() {
        telemetry.addData("pivotPower", getPivotPower());

        telemetryPacket.put("pivotPower", getPivotPower());
    }

    private double clamp(double power) {
        if (power > 1) return 1;
        if (power < -1) return -1;
        return power;
    }

    public void setPivotPower(double power) {
        pivot.setPower(clamp(power));
    }

    public double getPivotPower() {
        return pivot.getPower();
    }
}
