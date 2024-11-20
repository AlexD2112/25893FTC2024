/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode V2", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive drive;
    private DcMotor liftDrive = null;

    private CRServo leftServo = null;
    private CRServo rightServo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        liftDrive = hardwareMap.get(DcMotor.class, "lift");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward
        liftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoder
        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // THESE CONSTANTS
        // PID Constants (adjust these values during testing)
//        double Kp = 0.3;
//        double Ki = 0.005;
//        double Kd = 0.9;
        double Kp = 0.01;
        double Ki = 0;
        double Kd = 0.001;
        //<<>>

        // PID variables
        double lastError = 0;
        double integral = 0;
        double output = 0;

        boolean registered = false;
        boolean braking = false;
        int targetPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speed = 0.75;

            // Drivetrain control using Road Runner
            double axial = -gamepad1.left_stick_y;  // Forward/backward
            double lateral = -gamepad1.left_stick_x;  // Strafing
            double yaw = - gamepad1.right_stick_x;  // Rotation
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

            // Intake control
            double intakePower = gamepad1.right_stick_y;
            //leftServo.setPower(intakePower * 2);
            //rightServo.setPower(intakePower * 2);

            // Lift control
            double liftDown = -gamepad1.left_trigger;
            double liftUp = gamepad1.right_trigger;

            if (gamepad1.y && !braking) {
                liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftDrive.setPower(1);
                braking = true;
            } else if (gamepad1.y && braking) {
                braking = false;
            }

            if (gamepad1.a) {
                leftServo.setPower(-1);
                rightServo.setPower(-1);
            } else if (gamepad1.x) {
                leftServo.setPower(1);
                rightServo.setPower(1);
            } else {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }

            int limiter = 1000;

            final int hardstop = 1600;

            if ((liftDrive.getCurrentPosition() < hardstop) && liftUp != 0 && !braking) {
                liftDrive.setPower(liftUp * 0.75);
                registered = false;
            } else if ((liftDrive.getCurrentPosition() < hardstop) && liftDown != 0 && !braking) {
                liftDrive.setPower(liftDown * 0.75);
                registered = false;
            } else if (!braking) {
                if (!registered) {
                    targetPosition = Math.min(liftDrive.getCurrentPosition(), hardstop);
                    registered = true;
                }

                // PID Control for lift
                int currentPosition = liftDrive.getCurrentPosition();
                double error = targetPosition - currentPosition;

                double proportional = Kp * error;
                integral += error;
                double integralTerm = Ki * integral;
                double derivative = error - lastError;
                double derivativeTerm = Kd * derivative;

                output = proportional + integralTerm + derivativeTerm;
                liftDrive.setPower(output);

                lastError = error;
            }

            // Telemetry updates
            telemetry.addData("Lift Target", targetPosition);
            telemetry.addData("Lift Current", liftDrive.getCurrentPosition());
            telemetry.addData("Lift Output", output);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }}
