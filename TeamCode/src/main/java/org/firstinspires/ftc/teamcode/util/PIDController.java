package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Calendar;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private double previousError = 0;
    private double integralSum = 0;
    private double outputLimit = 1.0; // Example: Max output value
    private final TelemetryPacket packet = new TelemetryPacket();

    public PIDController(double Kp, double Ki, double Kd, double outputLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputLimit = outputLimit;
    }

    public double calculate(double error, double deltaTime) {
        double proportional = calculateP(error);
        double integral = calculateI(error, deltaTime);
        double derivative = calculateD(error, deltaTime);

        double output = proportional + integral + derivative;

        // Apply output limits if necessary
        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;

        previousError = error;
        return output;
    }

    public double calculateP(double error) { return Kp * error; }
    public double calculateI(double error, double deltaTime) {
        integralSum += error * deltaTime;
        return Ki * integralSum;
    }
    public double calculateD(double error, double deltaTime) {return Kd * ((error - previousError) / deltaTime); }
}
