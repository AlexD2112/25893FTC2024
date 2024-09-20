package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@TeleOp(name = "MecanumFixed", group = "Concept")
//@Disabled
public class DriveAndVision extends LinearOpMode {

    // CONTROL decleration
    public boolean isVision = true;

    // Motor Declerations
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RFMotor = null;
    private DcMotor RBMotor = null;
    private DcMotor LFMotor = null;
    private DcMotor lBMotor = null;

    // Vision Declerations
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // MOTOR STUFF
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RFMotor  = hardwareMap.get(DcMotor.class, "RF");
        RBMotor = hardwareMap.get(DcMotor.class, "RB");
        LFMotor = hardware.get(DcMotor.class, "LF");
        LBMotor = hardware.get(DcMotor.class, "LB")
       
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);

        // VISION STUFF
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (isVision == false) {
                    // MOTORS
                    double vertical = 0;
                    double horizontal = 0;
                    double pivot = 0;

                    // Driving in mecanum mode
                    vertical = -gamepad1.left_stick_y;
                    horizontal = gamepad1.left_stick_x * 1.1;
                    pivot = gamepad1.right_stick_x;
                
                    // Send calculated power to wheels
                    double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);
                    double flp = (vertical + horizontal + pivot) / denominator;
                    double blp = (vertical - horizontal + pivot) / denominator;
                    double frp = (vertical - horizontal - pivot) / denominator;
                    double brp = (vertical + horizontal - pivot) / denominator;
                    RFMotor.setPower(frp);
                    RBMotor.setPower(brp);
                    LFMotor.setPower(flp);
                    LBMotor.setPower(blp);

                    // Show the elapsed game time and wheel power.
                    telemetry.addData("isVision", isVision);
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                    telemetry.update();

                    if (gamepad1.a) {
                        isVision = true;
                    }
                } else if (isVision == true) {
                    double yaw = telemetryAprilTag();
                    telemetryAprilTag();

                    telemetry.addData("isVision", isVision);
                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // adjust pitch
                    /*
                    if (yaw > 0.5) {
                        leftDrive.setPower(0.2);
                        rightDrive.setPower(-0.2);
                    } else if (yaw < -0.5) {
                        leftDrive.setPower(-0.2);
                        rightDrive.setPower(0.2);
                    } else {
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                    }
                    */

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }
                   
                    if (gamepad1.b) {
                        isVision = false;
                    }

                    // Share the CPU.
                    sleep(20);
                }
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
            visionPortal.resumeLiveView();
            visionPortal.setCameraResolution(new Size(176, 120));
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private double telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
               
                if (currentDetections.indexOf(detection) == 0) {
                    return detection.ftcPose.yaw;
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
       
        return 0;

    }   // end method telemetryAprilTag()

} // end class ConceptAprilTag 