package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MoveArm", group = "Linear Opmode")
public class MoveArm extends LinearOpMode {

    // Declare motor
    private DcMotor motor, lift;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        motor = hardwareMap.get(DcMotor.class, "slider");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Set the motor to brake when power is zero
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Check if the "B" button is pressed
            if (gamepad1.b) {
                // Move the motor forward slowly
                motor.setPower(0.8);
            } else if (gamepad1.x) {
                motor.setPower(-0.8);
            } else {
                motor.setPower(0);
            }

            if (gamepad1.y) {
                lift.setPower(1);
            } else if (gamepad1.a) {
                lift.setPower(-1);
            } else {
                lift.setPower(0);
            }

            // Add telemetry data to see the motor power
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
