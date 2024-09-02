package org.firstinspires.ftc.robotcontroller;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Detection & Controller Light", group = "Sensor")
public class BgChange extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() {
        // Initialize the camera and vision processor
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        // Wait for the start of the match
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            if (!detections.isEmpty()) {
                // Telemetry update for tag detected
                telemetry.addLine("+-------------------------+");
                telemetry.addLine("|   ____    _  __        |");
                telemetry.addLine("|  / __ \\  | |/ /        |");
                telemetry.addLine("| | |  | | | ' /         |");
                telemetry.addLine("| | |  | | |  <          |");
                telemetry.addLine("| | |__| | | . \\         |");
                telemetry.addLine("|  \\____/  |_|\\        |");
                telemetry.addLine("+-------------------------+");
                telemetry.addData("AprilTag Detected", "Yes");
                telemetry.addData("Number of Tags", detections.size());

                // Set controller LED light to green
                gamepad1.setLedColor(0, 1, 0, 1000);  // Green color for 1 second

                // Rumble the gamepad for 2.5 seconds
                gamepad1.rumble((int) 2500);
            } else {
                // Telemetry update for no tag detected
                telemetry.addLine("+-------------------------+");
                telemetry.addLine("|  _   _  ____   ____     |");
                telemetry.addLine("| | \\ | |/ __ \\ / __ \\    |");
                telemetry.addLine("| |  \\| | |  | | |  | |   |");
                telemetry.addLine("| | . ` | |  | | |  | |   |");
                telemetry.addLine("| | |\\  | |__| | |__| |   |");
                telemetry.addLine("| |_| \\_\\\\____/ \\____/    |");
                telemetry.addLine("|                         |");
                telemetry.addLine("+-------------------------+");
                telemetry.addData("AprilTag Detected", "No");
                telemetry.addData("Number of Tags", 0);

                // Set controller LED light to red
                gamepad1.setLedColor(1, 0, 0, 1000);  // Red color for 1 second
            }

            telemetry.update();
        }
    }
}
