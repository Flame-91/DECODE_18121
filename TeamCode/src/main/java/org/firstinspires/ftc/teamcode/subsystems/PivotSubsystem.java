package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PivotSubsystem extends SubsystemBase {
    private final DcMotor pivot;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    private final double encoderTicksM = 5;
    private final double encoderTicksB = 5;

    public PivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        pivot = hardwareMap.get(DcMotor.class, "pivotMotor");
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
    }

    @Override
    public void periodic() {
        telemetry.addData("pivotPower", getPivotPower());
        telemetry.addData("currentPivotPosition", getCurrentPivotPosition());
        telemetry.addData("targetPivotPosition", getTargetPivotPosition());

        telemetryPacket.put("pivotPower", getPivotPower());
        telemetryPacket.put("currentPivotPosition", getCurrentPivotPosition());
        telemetryPacket.put("targetPivotPosition", getTargetPivotPosition());
    }

    public void setPivotPower(double power) {
        pivot.setPower(power);
    }
    public void setPivotPosition(int position) {
        pivot.setTargetPosition(position);
    }
    public double convertPivotTicksToAngle(double ticks) {
        return (ticks - encoderTicksB)/encoderTicksM;
    }

    public int convertPivotAngleToTicks(double angle) {
        return (int) Math.floor((angle * encoderTicksM) + encoderTicksB);
    }

    public double getCurrentPivotPosition() { return pivot.getCurrentPosition(); }
    public double getTargetPivotPosition() { return pivot.getTargetPosition(); }
    public double getPivotPower() {
        return pivot.getPower();
    }
}
