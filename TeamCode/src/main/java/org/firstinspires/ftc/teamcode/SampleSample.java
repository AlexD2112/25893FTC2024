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
@Autonomous(name = "Sample Sample", group = "Robot")
public class SampleSample extends LinearOpMode {
    @Override
    public void runOpMode() {
        Lift lift = new Lift(hardwareMap);
        lift.setTargetPosition(0);
        Pose2d initialPose = new Pose2d(0, -5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Extend extend = new Extend(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        TelemetryRunner telemetryRunner = new TelemetryRunner(drive, extend, intake, lift);

        TrajectoryActionBuilder moveReadyScore = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(9, 11), Math.toRadians(135));

        Pose2d readyScore = new Pose2d(new Vector2d(9, 11), Math.toRadians(135));

        TrajectoryActionBuilder moveIntoScore = drive.actionBuilder(readyScore)
                .strafeTo(new Vector2d(3.5, 20.5));

        Pose2d scoring = new Pose2d(new Vector2d(4, 20), Math.toRadians(135));

        TrajectoryActionBuilder backFromScore = drive.actionBuilder(scoring)
                .strafeTo(new Vector2d(9, 11));

        TrajectoryActionBuilder moveToGrab = drive.actionBuilder(readyScore)
                .turnTo(Math.toRadians(0));

        Pose2d readyNotFacing = new Pose2d(new Vector2d(9, 11), Math.toRadians(0));

        TrajectoryActionBuilder spinToScore = drive.actionBuilder(readyNotFacing)
                .turnTo(Math.toRadians(135))
                .strafeTo(new Vector2d(3.5, 20.5))
                .waitSeconds(0.2);

        TrajectoryActionBuilder moveToGrab2 = drive.actionBuilder(scoring)
                .strafeToLinearHeading(new Vector2d(9, 21), Math.toRadians(0));

        Pose2d readyNotFacingSecond = new Pose2d(new Vector2d(9, 21), Math.toRadians(0));

        TrajectoryActionBuilder spinToScore2 = drive.actionBuilder(readyNotFacingSecond)
                .turnTo(Math.toRadians(135))
                .strafeTo(new Vector2d(3.5, 20.5))
                .waitSeconds(0.2);

        TrajectoryActionBuilder spinGrab3 = drive.actionBuilder(scoring)
                .strafeTo(new Vector2d(10, 15))
                .turnTo(Math.toRadians(24))
                .strafeTo(new Vector2d(11.5, 17.4));

        Pose2d grabbed3 = new Pose2d(new Vector2d(11.5, 17.4), Math.toRadians(24));

        TrajectoryActionBuilder moveFinalScore = drive.actionBuilder(grabbed3)
                .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(135))
                .strafeTo(new Vector2d(3.5, 20.5));


        //13.32

        // Parallel action for arm control
        Action parallelArmControl = new ParallelAction(
                lift.runArmPID(),
                telemetryRunner.runTelemetry(),
                new SequentialAction(
                        telemetryRunner.waitAction(10),
                        new ParallelAction(
                                new SequentialAction(
                                        telemetryRunner.waitAction(0.4),
                                        moveReadyScore.build()),
                                lift.liftBasket(),
                                new SequentialAction(
                                        telemetryRunner.waitAction(0.7),
                                        extend.extendFully(true)),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.1),
                                        moveIntoScore.build()),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.8),
                                        lift.liftScoreBasket()),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.1),
                                        intake.release())
                        ),
                        new ParallelAction(
                                new SequentialAction(
                                        backFromScore.build(),
                                        moveToGrab.build()),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.5),
                                        lift.liftExtendedUp()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.9),
                                        lift.liftExtendedNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.4),
                                        lift.liftNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.8),
                                        lift.stopPID()
                                )
                        ),
                        new ParallelAction(
                                intake.intake(),
                                new SequentialAction(telemetryRunner.waitAction(0.7), extend.retractFully(false)),
                                new SequentialAction(telemetryRunner.waitAction(0.9), lift.liftBasket()),
                                new SequentialAction(telemetryRunner.waitAction(1.4), moveIntoScore.build()),
                                new SequentialAction(telemetryRunner.waitAction(2.1), extend.extendFully(true)),
                                new SequentialAction(telemetryRunner.waitAction(3.7), lift.liftScoreBasket()),
                                new SequentialAction(telemetryRunner.waitAction(4), intake.release())
                        ),
                        new ParallelAction(
                                new SequentialAction(
                                        moveToGrab2.build()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.5),
                                        lift.liftExtendedUp()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.9),
                                        lift.liftExtendedNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.4),
                                        lift.liftNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.8),
                                        lift.stopPID()
                                )
                        ),
                        new ParallelAction(
                                intake.intake(),
                                new SequentialAction(telemetryRunner.waitAction(0.7), extend.retractFully(false)),
                                new SequentialAction(telemetryRunner.waitAction(0.9), lift.liftBasket()),
                                new SequentialAction(telemetryRunner.waitAction(1.4), moveIntoScore.build()),
                                new SequentialAction(telemetryRunner.waitAction(2.1), extend.extendFully(true)),
                                new SequentialAction(telemetryRunner.waitAction(3.7), lift.liftScoreBasket()),
                                new SequentialAction(telemetryRunner.waitAction(4), intake.release())
                        ),
                        new ParallelAction(
                                lift.liftFullUp(),
                                new SequentialAction(
                                        telemetryRunner.waitAction(0.3),
                                        extend.retractFully(false),
                                        telemetryRunner.waitAction(0.3),
                                        extend.extendFully()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(0.2),
                                        spinGrab3.build()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.5),
                                        lift.liftExtendedUp()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(1.9),
                                        lift.liftExtendedNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.4),
                                        lift.liftNeutral()
                                ),
                                new SequentialAction(
                                        telemetryRunner.waitAction(2.8),
                                        lift.stopPID()
                                )
                        ),
                        new ParallelAction(
                                intake.intake(),
                                new SequentialAction(telemetryRunner.waitAction(0.7), extend.retractFully(false)),
                                new SequentialAction(telemetryRunner.waitAction(0.9), lift.liftFullUp()),
                                new SequentialAction(telemetryRunner.waitAction(1.4), moveFinalScore.build()),
                                new SequentialAction(telemetryRunner.waitAction(2.1), extend.extendFully(true)),
                                new SequentialAction(telemetryRunner.waitAction(3.7), lift.liftScoreBasket()),
                                new SequentialAction(telemetryRunner.waitAction(4), intake.release())
                        )
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

        Actions.runBlocking(parallelArmControl);
//        Actions.runBlocking(new ParallelAction(
//                lift.runArmPID(),
//                telemetryRunner.runTelemetry(),
//                new SequentialAction(
//                    lift.liftNeutral(),
//                    lift.liftBasket(),
//                        lift.liftNeutral(),
//                        lift.liftScoreBasket()
//                ))
//        );
        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }

    public class TelemetryRunner {
        MecanumDrive drive;
        Extend extend;
        Intake intake;
        Lift lift;

        public TelemetryRunner(MecanumDrive drive, Extend extend, Intake intake, Lift lift) {
            this.drive = drive;
            this.extend = extend;
            this.intake = intake;
            this.lift = lift;
        }

        public Action runTelemetry() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket packet) {
                    Pose2d currentPose = drive.pose;

                    telemetry.addData("Robot Pos", currentPose.position);
                    telemetry.addData("Robot Heading", currentPose.heading);
                    telemetry.addData("Lift Target", lift.getTargetPosition());
                    telemetry.addData("Lift Current", lift.getCurrentPosition());
                    telemetry.addData("Lift P/I/D", lift.proportional + " | " + lift.integral + " | " + lift.derivativeTerm);
                    telemetry.addData("Lift I", lift.integralTerm);
                    telemetry.addData("Lift D", lift.derivativeTerm);
                    telemetry.update();

                    packet.put("Lift Current", lift.getCurrentPosition());
                    packet.put("Lift Target", lift.getTargetPosition());

// You can do the same for the terms
                    packet.put("P-Term", lift.proportional);
                    packet.put("I-Term", lift.integralTerm);
                    packet.put("D-Term", lift.derivativeTerm);


                    return true; // Keep running
                }
            };
        }
        public Action waitAction(double seconds) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                    }

                    if (timer.seconds() >= seconds) {
                        return false; // Action complete
                    }

                    return true; // Keep running
                }
            };
        }
    }

    public class Lift {
        private final DcMotorEx lift;
        private double lastError = 0;
        private double integral = 0;
        private double lastOutput = 0;

        // PID Constants


        // Target positions
        private final double NEUTRAL_POSITION = -250;
        private final double EXTENDED_DOWN_POSITION = -150;
        private final double EXTENDED_NEUTRAL_POSITION = -350;
        private final double LOW_POSITION = -50;
        private final double HIGH_POSITION = -420;
        private final double SCORE_POSITION = -480;
        private final double UP_POSITION = -880;
        private final double BASKET_POSITION = -840; //UNTESTED
        private final double SCORE_BASKET_POSITION = -800; //UNTESTED
        private final double EXTENDED_UP_POSITION = -640;
        private final double FULL_UP_POSITION = -910;
        public double integralTerm;
        public double derivativeTerm;
        public double proportional;

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
            proportional = LiftConfig.Kp * error;
            integral *= 0.95;
            integral += error;
            integralTerm = LiftConfig.Ki * integral;
            double derivative = error - lastError;
            derivativeTerm = LiftConfig.Kd * derivative;

            lastOutput = proportional + integralTerm - derivativeTerm;
            lift.setPower(lastOutput);
//            lift.setPower(lastOutput < 0 ? Math.min(Math.max(lastOutput, -0.5), -0.0005) : Math.min(lastOutput, 0.6));

            lastError = error;
        }

        public void setTargetPosition(double position) {
            targetPosition = position;
            activePID = true; // Ensure PID becomes active when setting a new target
        }


        public Action stopPID() {
            return new Action() {
                @Override
                public boolean run(TelemetryPacket packet) {
                    activePID = false;
                    lift.setPower(0); // Stop the motor
                    return false; // Action completes immediately
                }
            };
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
        public Action liftHigh()  {
            return createLiftAction(HIGH_POSITION);
        }

        public Action liftLow()  {
            return createLiftAction(LOW_POSITION);
        }

        public Action liftExtendedDown() {
            return createLiftAction(EXTENDED_DOWN_POSITION);
        }

        public Action liftExtendedNeutral() {
            return createLiftAction(EXTENDED_NEUTRAL_POSITION);
        }

        public Action liftBasket() {
            return createLiftAction(BASKET_POSITION);
        }

        public Action liftScoreBasket() {
            return createLiftAction(SCORE_BASKET_POSITION);
        }
        public Action liftExtendedUp() { return createLiftAction(EXTENDED_UP_POSITION); }
        public Action liftFullUp() { return createLiftAction(FULL_UP_POSITION); }

        public double getTargetPosition() {
            return targetPosition;
        }
        public double getCurrentPosition() {
            return lift.getCurrentPosition();
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
                        integral = 0;
                    }

                    if (timer.seconds() >= 1.2) {
                        return false; // Action completes after 1.2 seconds
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

        public Action retractExtend() {
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

                    if (timer.milliseconds() >= RETRACT_DURATION_MS + EXTEND_DURATION_MS) {
                        extendMotor.setPower(0);
                        return false; //Action complete
                    } else if (timer.milliseconds() >= RETRACT_DURATION_MS) {
                        extendMotor.setPower(EXTEND_POWER); // Stop motor after time elapses
                        return true; // Keep running
                    }


                    return true; // Continue running
                }
            };
        }

        public Action extendFully() {
            return extendFully(false);
        }

        public Action extendFully(boolean stretchHard) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();
                private double extend_duration = stretchHard ? EXTEND_DURATION_MS * 1.3 : EXTEND_DURATION_MS;

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

                    if (timer.milliseconds() >= extend_duration) {
                        extendMotor.setPower(0); // Stop motor after time elapses
                        return false; // Action complete
                    }

                    return true; // Continue running
                }
            };
        }

        public Action retractFully(boolean delayed) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        if (delayed) {
                            extendMotor.setPower(0);
                        } else {
                            extendMotor.setPower(RETRACT_POWER);
                        }
                    }

                    if (delayed) {
                        if (timer.milliseconds() >= RETRACT_DURATION_MS * 1.8) {
                            extendMotor.setPower(0);
                            return false;
                        }
                        if (timer.milliseconds() >= RETRACT_DURATION_MS * 0.8) {
                            extendMotor.setPower(RETRACT_POWER); // Stop motor after time elapses
                            return true; // Action running
                        }
                    } else {
                        if (timer.milliseconds() >= RETRACT_DURATION_MS) {
                            extendMotor.setPower(0);
                            return false;
                        }
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

        // 5-second duration for intake/release
        private static final long INTAKE_DURATION_MS = 520;
        private static final long DROP_DURATION_MS = 1100;

        public Intake(HardwareMap hardwareMap) {
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");

            // Set directions for servos (adjust based on your setup)
            leftServo.setDirection(CRServo.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        public Action intake() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        // Start intake
                        leftServo.setPower(INTAKE_POWER);
                        rightServo.setPower(INTAKE_POWER);
                    }

                    // Send telemetry data if desired
                    packet.put("Intake Status", "Running");
                    packet.put("Intake Elapsed Time (ms)", timer.milliseconds());

                    // After 5 seconds, stop the servos and complete the action
                    if (timer.milliseconds() >= INTAKE_DURATION_MS) {
                        leftServo.setPower(0);
                        rightServo.setPower(0);
                        packet.put("Intake Status", "Complete");
                        return false; // Action complete
                    }

                    return true; // Keep running until 5 seconds pass
                }
            };
        }

        public Action release() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime timer = new ElapsedTime();

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        initialized = true;
                        timer.reset();
                        // Start releasing
                        leftServo.setPower(RELEASE_POWER);
                        rightServo.setPower(RELEASE_POWER);
                    }

                    // Send telemetry data if desired
                    packet.put("Release Status", "Running");
                    packet.put("Release Elapsed Time (ms)", timer.milliseconds());

                    // After 5 seconds, stop the servos and complete the action
                    if (timer.milliseconds() >= DROP_DURATION_MS) {
                        leftServo.setPower(0);
                        rightServo.setPower(0);
                        packet.put("Release Status", "Complete");
                        return false; // Action complete
                    }

                    return true; // Keep running until 5 seconds pass
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
