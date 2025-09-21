package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double setpoint;
    private double previousError = 0;
    private double integralSum = 0;
    private double outputLimit = 1.0; // Example: Max output value

    public PIDController(double Kp, double Ki, double Kd, double setpoint, double outputLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.setpoint = setpoint;
        this.outputLimit = outputLimit;
    }

    public double calculate(double processVariable, double deltaTime) {
        double error = setpoint - processVariable;

        integralSum += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Apply output limits if necessary
        output = Math.max(-outputLimit, Math.min(outputLimit, output));

        previousError = error;
        return output;
    }

    public void setSetpoint(double newSetpoint) {
        this.setpoint = newSetpoint;
        // Reset integral sum when setpoint changes to prevent wind-up
        this.integralSum = 0;
    }

    // Other methods for setting Kp, Ki, Kd, etc.
}
