package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Random;

@Config
@Autonomous(name = "Two Piece Auto", group = "Robot")
public class TwoPieceAuto extends LinearOpMode {
    private double velocity = 0;

    @Override
    public void runOpMode() {
        Lift lift = new Lift(hardwareMap); // Initialize the Lift

        waitForStart(); // Wait for the game to start

        if (opModeIsActive()) {
            Random random = new Random(); // Random generator
            String[] actions = {"up", "neutral", "down"}; // Possible lift actions

            for (int i = 0; i < 6; i++) { // Run a few random combinations
                String action = actions[random.nextInt(actions.length)];
                telemetry.addData("Action", action); // Display the chosen action
                telemetry.update();

                Action liftAction;
                switch (action) {
                    case "up":
                        liftAction = lift.liftUp();
                        break;
                    case "neutral":
                        liftAction = lift.liftNeutral();
                        break;
                    case "down":
                        liftAction = lift.liftDown();
                        break;
                    default:
                        liftAction = null; // Fallback, shouldn't happen
                }

                while (opModeIsActive() && liftAction.run(new TelemetryPacket())) {
                    telemetry.addData("Action In Progress", action);
                    telemetry.addData("Current Encoder Position", lift.getCurrentPosition());
                    telemetry.addData("Target Position", lift.getTargetPosition());
                    telemetry.addData("Motor Power", lift.getMotorPower());
                    telemetry.addData("Velocity", velocity);
                    telemetry.addData("PID Error", lift.getLastPIDError());
                    telemetry.addData("PID Output", lift.getLastPIDOutput());
                    telemetry.update();
                }

                sleep(500); // Pause for half a second between actions
            }
        }
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
        private final double UP_POSITION = -700;
        private final double DOWN_POSITION = -100;
        private double targetPosition = 0;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        public class LiftToPosition implements Action {
            private final double position;
            private boolean initialized = false;

            public LiftToPosition(double position) {
                this.position = position;
            }

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    targetPosition = position; // Set target position
                }

                double currentPosition = lift.getCurrentPosition();
                double error = position - currentPosition;

                // Check if target position is reached
                if (Math.abs(error) < 10) {
                    lift.setPower(0); // Stop the motor
                    return false; // Action complete
                }

                // Run PID control
                runPID(currentPosition, position, packet);
                return true; // Continue running
            }

            private void runPID(double currentPosition, double targetPosition, TelemetryPacket packet) {
                double error = targetPosition - currentPosition;

                // PID calculations
                double proportional = Kp * error;
                integral += error;
                double integralTerm = Ki * integral;
                double derivative = error - lastError;
                double derivativeTerm = Kd * derivative;

                lastOutput = proportional + integralTerm + derivativeTerm;
                lift.setPower(lastOutput);

                lastError = error;

                // Add telemetry for debugging
                packet.put("PID Error", error);
                packet.put("PID Output", lastOutput);
            }
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    targetPosition = DOWN_POSITION;
                }

                double currentPosition = lift.getCurrentPosition();
                double error = DOWN_POSITION - currentPosition;

                if (Math.abs(error) < 10) {
                    lift.setPower(0); // Stop the motor
                    return false; // Action complete
                }

                // Run PID control for downward motion
                runPID(currentPosition, DOWN_POSITION, packet);
                return true; // Continue running
            }

            private void runPID(double currentPosition, double targetPosition, TelemetryPacket packet) {
                double error = targetPosition - currentPosition;

                // PID calculations
                double proportional = Kp * error;
                integral += error;
                double integralTerm = Ki * integral;
                double derivative = error - lastError;
                double derivativeTerm = Kd * derivative;

                lastOutput = proportional + integralTerm + derivativeTerm;
                lift.setPower(lastOutput);

                lastError = error;

                // Add telemetry for debugging
                packet.put("PID Error", error);
                packet.put("PID Output", lastOutput);
            }
        }

        public Action liftUp() {
            return new LiftToPosition(UP_POSITION);
        }

        public Action liftNeutral() {
            return new LiftToPosition(NEUTRAL_POSITION);
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }
}
