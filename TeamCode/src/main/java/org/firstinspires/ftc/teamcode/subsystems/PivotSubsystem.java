package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotTicksAtFortyFive;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotTicksAtNinety;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class PivotSubsystem extends SubsystemBase {
    private final DcMotor leftPivotMotor;
    private final DcMotor rightPivotMotor;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public static double offsetAngleFromLimelightToPivot = -5;
    public PivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        leftPivotMotor = hardwareMap.get(DcMotor.class, "leftPivotMotor");
        rightPivotMotor = hardwareMap.get(DcMotor.class, "rightPivotMotor");
        leftPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("pivotPower", getPivotPower());
        telemetry.addData("currentPivotPositionAngle", getCurrentPivotPositionAngle());
        telemetry.addData("currentPivotPositionTicks", getCurrentPivotPositionTicks());

        telemetryPacket.put("pivotPower", getPivotPower());
        telemetryPacket.put("currentPivotPositionAngle", getCurrentPivotPositionAngle());
        telemetryPacket.put("currentPivotPositionTicks", getCurrentPivotPositionTicks());
    }

    public void setPivotPower(double power) {
        leftPivotMotor.setPower(power);
        rightPivotMotor.setPower(power);
    }

    public void movePivotWithoutEncoder(double power) {
        leftPivotMotor.setPower(power);
        rightPivotMotor.setPower(power);
    }
    public void resetPivotEncoder() { // Should only be called if pivot is at rest DOWN
        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double convertPivotTicksToAngle(double ticks) {
        return ticks*(45 / (pivotTicksAtNinety-pivotTicksAtFortyFive));
    }

    public int convertPivotAngleToTicks(double angle) {
        return (int) ((int) angle*((pivotTicksAtNinety-pivotTicksAtFortyFive) / 45));
    }

    public double getCurrentPivotPositionTicks() { return leftPivotMotor.getCurrentPosition()  - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot); }
    public double getCurrentPivotPositionAngle() { return convertPivotTicksToAngle(leftPivotMotor.getCurrentPosition()) - offsetAngleFromLimelightToPivot; }
    //    public double getTargetPivotPosition() { return leftPivotMotor.getTargetPosition()  - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot); }
    public double getPivotPower() { return leftPivotMotor.getPower(); }
}
