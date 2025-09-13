package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    private double Kp; // Proportional gain
    private double Ki; // Integral gain
    private double Kd; // Derivative gain

    private double setpoint; // Desired target value
    private double integralSum; // Accumulation of errors over time
    private double lastError; // Error from the previous calculation
    private double lastTime; // Time of the previous calculation

    // Constructor to initialize the PID gains
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.integralSum = 0;
        this.lastError = 0;
        this.lastTime = System.nanoTime(); // Initialize with current time
    }

    // Set the desired target value
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Calculate the controller output based on the current process variable
    public double calculate(double currentValue) {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1_000_000_000.0; // Convert nanoseconds to seconds

        // Calculate the error
        double error = setpoint - currentValue;

        // Calculate the proportional term
        double proportionalTerm = Kp * error;

        // Calculate the integral term
        integralSum += error * deltaTime;
        double integralTerm = Ki * integralSum;

        // Calculate the derivative term
        double derivative = (error - lastError) / deltaTime;
        double derivativeTerm = Kd * derivative;

        // Calculate the total output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Update for the next iteration
        lastError = error;
        lastTime = currentTime;

        return output;
    }

    // Reset the integral sum and last error (useful when changing setpoints or restarting)
    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }
}
