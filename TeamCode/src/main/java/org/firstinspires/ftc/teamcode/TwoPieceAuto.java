package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
@Autonomous(name = "Two Piece Auto", group = "Robot")
public class TwoPieceAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Extend extend = new Extend(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Predefined lift and intake actions
        // Predefined lift and intake actions with telemetry
        Action liftToNeutral = new SequentialAction(
                lift.liftNeutral(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Lift", "Neutral Position");
                        telemetry.update();
                        return false;
                    }
                }
        ); // Adjust for neutral position
        Action liftFullyUp = new SequentialAction(
                lift.liftUp(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Lift", "Fully Up");
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action liftScore = new SequentialAction(
                lift.liftScore(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Lift", "Score Position");
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action liftFullyUp2 = new SequentialAction(
                lift.liftUp(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Lift", "Fully Up");
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action release = new SequentialAction(
                intake.release(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Intake", "Release Complete");
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action stretch = new SequentialAction(
                extend.extendFully(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Extend", "Fully Extended");
                        telemetry.update();
                        return false;
                    }
                }
        );

        // Trajectory actions using splines (straight Y-axis movement)
        // Trajectory actions using splines (straight Y-axis movement) with telemetry
        Action moveForward1 = new SequentialAction(
                drive.actionBuilder(initialPose)
                        .splineTo(new Vector2d(16, 12), Math.toRadians(135))
                        .build(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Trajectory", "Move Forward 1 Foot");
                        telemetry.addData("Current Pose", drive.pose);
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action moveForward2 = new SequentialAction(
                drive.actionBuilder(new Pose2d(18, 12, Math.toRadians(135)))
                        .splineTo(new Vector2d(2, 28), Math.toRadians(135))
                        .build(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Trajectory", "Move Forward 2 Feet");
                        telemetry.addData("Current Pose", drive.pose);
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action moveBack = new SequentialAction(
                drive.actionBuilder(new Pose2d(0, 24, Math.toRadians(90)))
                        .splineTo(new Vector2d(0, 12), Math.toRadians(90))
                        .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                        .build(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Trajectory", "Move Back to Neutral");
                        telemetry.addData("Current Pose", drive.pose);
                        telemetry.update();
                        return false;
                    }
                }
        );

        Action finalMove = new SequentialAction(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                        .splineTo(new Vector2d(0, 12), Math.toRadians(90))
                        .build(),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        telemetry.addData("Trajectory", "Final Move Forward 1 Foot");
                        telemetry.addData("Current Pose", drive.pose);
                        telemetry.update();
                        return false;
                    }
                }
        );

        // Parallel PID controller action
        Action parallelPID = lift.runArmPID();

        // Main autonomous sequence
        Action mainSequence = new SequentialAction(
                liftToNeutral,
                moveForward1,
                liftFullyUp,
                stretch,
                moveForward2,
                liftScore,
                liftFullyUp2,
                release
        );

        // Combined autonomous action with parallel PID
        Action fullSequence = new ParallelAction(parallelPID, mainSequence);

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

        // Execute the pre-built sequence
        Actions.runBlocking(fullSequence);

        telemetry.addData("Status", "Autonomous complete");
        telemetry.addData("Final Pose", drive.pose);
        telemetry.update();
    }


    public class Lift {
        private final DcMotorEx lift;
        private double lastError = 0;
        private double integral = 0;
        private double lastOutput = 0;

        // PID Constants
        private final double Kp = 0.01;
        private final double Ki = 0.0000001;
        private final double Kd = 0.003;

        // Target positions
        private final double NEUTRAL_POSITION = -270;
        private final double SCORE_POSITION = -770;
        private final double UP_POSITION = -880;
        private final double DOWN_POSITION = -50;
        private double targetPosition = 0;

        private boolean activePID = true; // Tracks if the PID is active

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
            lift.setPower(lastOutput);

            //Add telemetry
            telemetry.addData("Lift Target", targetPosition);
            telemetry.addData("Lift Current", currentPosition);
            telemetry.addData("Lift Output", lastOutput);
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

        public double getCurrentPosition() {
            return lift.getCurrentPosition();
        }

        public double getTargetPosition() {
            return targetPosition;
        }

        public double getMotorPower() {
            return lift.getPower();
        }

        public double getLastPIDError() {
            return lastError;
        }

        public double getLastPIDOutput() {
            return lastOutput;
        }

        public Action runArmPID() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket packet) {
                    updatePID();
                    return true; // Continues indefinitely
                }
            };
        }

        // Lift actions with delay
        public Action liftUp() {
            return new Action() {
                private ElapsedTime elapsedTime = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    setTargetPosition(UP_POSITION);

                    if (elapsedTime.seconds() >= 1.0) {
                        return false; // Action complete after 1 second
                    }

                    return true; // Keep running
                }
            };
        }

        public Action liftScore() {
            return new Action() {
                private ElapsedTime elapsedTime = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    setTargetPosition(SCORE_POSITION);

                    if (elapsedTime.seconds() >= 1.0) {
                        return false; // Action complete after 1 second
                    }

                    return true; // Keep running
                }
            };
        }

        public Action liftNeutral() {
            return new Action() {
                private ElapsedTime elapsedTime = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    setTargetPosition(NEUTRAL_POSITION);

                    if (elapsedTime.seconds() >= 1.0) {
                        return false; // Action complete after 1 second
                    }

                    return true; // Keep running
                }
            };
        }

        public Action liftDown() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime elapsedTime = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    setTargetPosition(DOWN_POSITION);

                    if (elapsedTime.seconds() >= 1.0) {
                        stopPID(); // Stop PID after 1 second
                        return false; // Action complete
                    }

                    return true; // Keep running
                }
            };
        }
    }

    public class Extend {
        private final DcMotor extendMotor;
        private final double EXTEND_POWER = 0.8; // Power for extending
        private final double RETRACT_POWER = -0.8; // Power for retracting
        private final long EXTEND_DURATION_MS = 1400; // Duration for full extension in milliseconds
        private final long RETRACT_DURATION_MS = 1400; // Duration for full retraction in milliseconds

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

                    // Telemetry for extending
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

                    // Telemetry for retracting
                    telemetry.addData("Extend Status", "Retracting");
                    telemetry.addData("Elapsed Time", timer.milliseconds());
                    telemetry.update();

                    if (timer.milliseconds() >= RETRACT_DURATION_MS) {
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
}
