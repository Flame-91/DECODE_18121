package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double previousError = 0;
    private double integralSum = 0;
    private double outputLimit = 1.0; // Example: Max output value
    private final TelemetryPacket packet = new TelemetryPacket();
    private final FtcDashboard dashboard;

    public PIDController(double Kp, double Ki, double Kd, double outputLimit, FtcDashboard dashboard) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputLimit = outputLimit;
        this.dashboard = dashboard;
    }

    public double calculate(double error, double deltaTime) {
        integralSum += error * deltaTime;
        double proportional = Kp * error;
        double integral = Ki * integralSum;
        double derivative = Kd * ((error - previousError) / deltaTime);

        packet.put("kP Output", proportional);
        packet.put("kI Output", integral);
        packet.put("kD Output", derivative);
        dashboard.sendTelemetryPacket(packet);

        double output = proportional + integral + derivative;

        // Apply output limits if necessary
        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;

        previousError = error;
        return output;
    }

    // Other methods for setting Kp, Ki, Kd, etc.
}
