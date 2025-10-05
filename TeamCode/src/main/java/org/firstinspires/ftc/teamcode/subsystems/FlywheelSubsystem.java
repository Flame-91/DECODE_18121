package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

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
    private final ElapsedTime timer = new ElapsedTime();
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setFlywheelMotor(double power) {
        if (power < -1 || power > 1) return;
        telemetry.addData("Flywheel Motor Power", power);
        flywheelMotor.setPower(power);
    }

    public void setServosPower(double power) {
        if (power < -1 || power > 1) return;
        telemetry.addData("Left & Right Servo Power", power);
        rightServo.setPower(power);
        leftServo.setPower(power);
    }
}
