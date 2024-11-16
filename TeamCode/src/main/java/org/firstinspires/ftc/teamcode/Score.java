/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */



@Autonomous(name="Score", group="Robot")
public class Score extends LinearOpMode {

    private MecanumDrive drive;
    private DcMotor liftDrive = null;

    private CRServo leftServo = null;
    private CRServo rightServo = null;

    /* Declare OpMode members. */
    static final double     FORWARD_SPEED = 0.1;

    @Override
    public void runOpMode() {
        // Initialize MecanumDrive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        liftDrive = hardwareMap.get(DcMotor.class, "lift");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        // Define the axial (forward/backward), lateral (sideways), and yaw (rotation) speeds
        double axial = -0.2;    // Slow backward speed
        double lateral = 0.0;   // No lateral movement
        double yaw = 0.0;       // No rotation

        //Elapsed time tracks elapsed time (very important comment)
        ElapsedTime runtime = new ElapsedTime();

        double Kp = 0.01;
        double Ki = 0;
        double Kd = 0.001;
        //<<>>

        // PID variables
        double lastError = 0;
        double integral = 0;
        double output = 0;

        int targetPosition = 0;
        int finalTargetPosition = -1200;

        // Set drive powers to move the robot
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        // Run for 30 seconds
        while (opModeIsActive() && (runtime.seconds() < 7.0)) {
            drive.updatePoseEstimate();
            telemetry.addData("Status", "Driving Sideways");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }

        axial = 0.1;
        lateral = 0;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        while (opModeIsActive() && (runtime.seconds() < 7.8)) {
            drive.updatePoseEstimate();
            telemetry.addData("Status", "Backing Up");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }

        axial = 0;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        while (opModeIsActive() && (runtime.seconds() < 15)) {
            targetPosition = (int) ((finalTargetPosition) * (Math.min(runtime.seconds(), 14) - 7.8) / (14-7.8));

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
            drive.updatePoseEstimate();
            telemetry.addData("Status", "Lifting arm");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Lift Target", targetPosition);
            telemetry.addData("Lift Current", liftDrive.getCurrentPosition());
            telemetry.addData("Lift Output", output);
            telemetry.update();
        }

        targetPosition = 1350;

        while (opModeIsActive() && runtime.seconds() < 17) {
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
            drive.updatePoseEstimate();
            telemetry.addData("Status", "Lifting arm");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Lift Target", targetPosition);
            telemetry.addData("Lift Current", liftDrive.getCurrentPosition());
            telemetry.addData("Lift Output", output);
            telemetry.update();
        }

        axial = -0.1;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        while (opModeIsActive() && runtime.seconds() < 17.6) {
            telemetry.addData("Status", "Lining up");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
            telemetry.update();
        }

        axial = 0;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), yaw));

        while (opModeIsActive() && runtime.seconds() < 20) {
            leftServo.setPower(-1);
            rightServo.setPower(-1);
            telemetry.addData("Status", "Scoring");
            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", yaw);
        }

        leftServo.setPower(0);
        rightServo.setPower(0);

        while (opModeIsActive() && runtime.seconds() < 30.0) {
            // Stop the robot
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            telemetry.addData("Planning is not a Northside strength", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}
