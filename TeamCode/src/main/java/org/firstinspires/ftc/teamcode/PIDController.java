package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController {
    private double kP, kI, kD; // PID coefficients
    private double previousError = 0; // Previous error for derivative calculation
    private double integral = 0; // Integral term
    private double setpoint = 0; // Desired value

    // Constructor
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Set the PID coefficients
    public void setPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Setpoint - desired value (could be distance, angle, etc.)
    public void setPoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // Reset the PID controller (e.g., reset integral term and previous error)
    public void reset() {
        this.previousError = 0;
        this.integral = 0;
    }

    // Compute the PID output based on the current sensor value (e.g., motor position)
    public double calculate(double currentValue) {
        double error = setpoint - currentValue;

        // Proportional term
        double pTerm = kP * error;

        // Integral term
        integral += error; // Accumulate error for integral

        // Derivative term
        double dTerm = kD * (error - previousError);

        // PID output (proportional, integral, and derivative)
        double output = pTerm + (kI * integral) + dTerm;

        // Save current error for next calculation
        previousError = error;

        return output;
    }

    // Method to apply the PID output to a motor, assuming an ideal setpoint matching motor encoder values
    public void applyToMotor(DcMotor motor, double currentValue) {
        double pidOutput = calculate(currentValue);
        motor.setPower(pidOutput);
    }
}
