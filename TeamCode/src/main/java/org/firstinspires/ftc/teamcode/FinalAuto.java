package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Final Auto", group = "Robot")
public class FinalAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Lift lift = new Lift(hardwareMap);
        lift.setTargetPosition(0);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Extend extend = new Extend(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Predefined actions with telemetry
        Action liftToNeutral = lift.liftNeutral();
        Action liftToNeutral2 = lift.liftNeutral();
        Action liftFullyUp = lift.liftUp();
        Action liftScore = lift.liftScore();
        Action release = intake.release();
        Action stretch = extend.extendFully();
        Action compress = extend.retractFully();

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(12.5, 0))
                .waitSeconds(3);

        // Parallel action for arm control
        Action parallelArmControl = new ParallelAction(
                lift.runArmPID(),
                new SequentialAction(
                        liftToNeutral,
                        forward.build(),
                        stretch,
                        liftScore,
                        compress,
                        liftToNeutral2
                )
        );

        // Wait for start
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            telemetry.addData("Status", "Stop requested before start");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Running autonomous sequence");
        telemetry.update();

        // Continuous telemetry update for robot position
        while (opModeIsActive()) {
            Actions.runBlocking(parallelArmControl);
            Pose2d currentPose = drive.pose;
            telemetry.addData("Robot Pos", currentPose.position);
            telemetry.addData("Robot Heading", currentPose.heading);
            telemetry.update();
        }

        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }

    public class Lift {
        private final DcMotorEx lift;
        private double lastError = 0;
        private double integral = 0;
        private double lastOutput = 0;

        // PID Constants
        private final double Kp = 0.002;
        private final double Ki = 0;
        private final double Kd = 0;//0.03;

        // Target positions
        private final double NEUTRAL_POSITION = -390;
        private final double SCORE_POSITION = -490;
        private final double UP_POSITION = -880;
        private final double DOWN_POSITION = -50;
        private double targetPosition = 0;

        private boolean activePID = true; // Tracks if the PID is active

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            targetPosition = 0;
        }

        public void updatePID() {
            if (!activePID) {
                lift.setPower(0); // If PID is inactive, stop motor power
                return;
            }

            double currentPosition = lift.getCurrentPosition();
            double error = targetPosition - currentPosition;

            // PID calculations
            double proportional = Kp * error;
            integral += error;
            double integralTerm = Ki * integral;
            double derivative = error - lastError;
            double derivativeTerm = Kd * derivative;

            lastOutput = proportional + integralTerm + derivativeTerm;
            lift.setPower(lastOutput < 0 ? Math.min(Math.max(lastOutput, -0.5), -0.0005) : Math.min(lastOutput, 0.6));

            // Add telemetry
            telemetry.addData("Lift Target", targetPosition);
            telemetry.addData("Lift Current", currentPosition);
            telemetry.addData("Lift Output", lastOutput);
            telemetry.addData("D value", Kd * derivative);
            telemetry.update();

            lastError = error;
        }

        public void setTargetPosition(double position) {
            targetPosition = position;
            activePID = true; // Ensure PID becomes active when setting a new target
        }

        public void stopPID() {
            activePID = false; // Deactivates the PID controller
            lift.setPower(0); // Ensures the motor stops
        }

        public Action runArmPID() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                    }

                    updatePID();

                    return true; // Keep running
                }
            };
        }

        // Lift actions with 2-second delay enforcement
        public Action liftUp() {
            return createLiftAction(UP_POSITION);
        }

        public Action liftScore() {
            return createLiftAction(SCORE_POSITION);
        }

        public Action liftNeutral() {
            return createLiftAction(NEUTRAL_POSITION);
        }

        private Action createLiftAction(double position) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        setTargetPosition(position);
                    }

                    if (timer.seconds() >= 5.0) {
                        return false; // Action completes after 2 seconds
                    }

                    return true; // Keep running
                }
            };
        }
    }

    public class Extend {
        private final DcMotor extendMotor;
        private final double EXTEND_POWER = -0.8; // Power for extending
        private final double RETRACT_POWER = 0.8; // Power for retracting
        private final long EXTEND_DURATION_MS = 1000; // Duration for full extension in milliseconds
        private final long RETRACT_DURATION_MS = 1000; // Duration for full extension in milliseconds

        public Extend(HardwareMap hardwareMap) {
            extendMotor = hardwareMap.get(DcMotor.class, "extend");
            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public Action extendFully() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        extendMotor.setPower(EXTEND_POWER);
                    }

                    telemetry.addData("Extend Status", "Extending");
                    telemetry.addData("Elapsed Time", timer.milliseconds());
                    telemetry.update();

                    if (timer.milliseconds() >= EXTEND_DURATION_MS) {
                        extendMotor.setPower(0); // Stop motor after time elapses
                        return false; // Action complete
                    }

                    return true; // Continue running
                }
            };
        }

        public Action retractFully() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        extendMotor.setPower(RETRACT_POWER);
                    }

                    telemetry.addData("Extend Status", "Retracting");
                    telemetry.addData("Elapsed Time", timer.milliseconds());
                    telemetry.update();

                    if (timer.milliseconds() >= EXTEND_DURATION_MS) {
                        extendMotor.setPower(0); // Stop motor after time elapses
                        return false; // Action complete
                    }

                    return true; // Continue running
                }
            };
        }

        public Action stop() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    extendMotor.setPower(0); // Immediately stop the motor
                    telemetry.addData("Extend Status", "Stopped");
                    telemetry.update();
                    return false; // Action complete
                }
            };
        }
    }

    public class Intake {
        private final CRServo leftServo;
        private final CRServo rightServo;

        private final double INTAKE_POWER = -1.0; // Power for intake
        private final double RELEASE_POWER = 1.0; // Power for release

        public Intake(HardwareMap hardwareMap) {
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");

            // Set directions for servos (adjust based on your setup)
            leftServo.setDirection(CRServo.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        public Action intake() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftServo.setPower(INTAKE_POWER);
                    rightServo.setPower(INTAKE_POWER);
                    packet.put("Intake", "Running");
                    return false; // This action completes immediately
                }
            };
        }

        public Action release() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftServo.setPower(RELEASE_POWER);
                    rightServo.setPower(RELEASE_POWER);
                    packet.put("Release", "Running");
                    return false; // This action completes immediately
                }
            };
        }

        public Action stop() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftServo.setPower(0);
                    rightServo.setPower(0);
                    packet.put("Intake", "Stopped");
                    return false; // This action completes immediately
                }
            };
        }
    }

    // Additional classes for Lift and Intake would be similar to your original implementation
    // No major changes required as they were functioning correctly.
}
