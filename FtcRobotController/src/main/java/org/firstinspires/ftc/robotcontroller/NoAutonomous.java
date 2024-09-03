package org.firstinspires.ftc.robotcontroller;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class NoAutonomous extends LinearOpMode {

    // Robot Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightDrive = null;
    private DcMotorEx leftDrive = null;
    private DcMotorEx lifter1 = null;
    private CRServo StorageDoor1 = null;
    private CRServo DownStorageDoor1 = null;
    private CRServo DownStorageDoor2 = null;

    private Servo foodClawServo1 = null; // Servo for FoodClaw1
    private Servo foodClawServo2 = null; // Servo for FoodClaw2
    private DcMotorEx antena = null;    // New Core Hex Motor for Antena
    private ColorSensor colorSensor = null; // Color Sensor for detecting orange

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open

    // AprilTag Detection Variables
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // Servo position constants for food claws
    private static final double CLAW_OPEN_POSITION = 0.6;  // Example position to open claws
    private static final double CLAW_CLOSED_POSITION = 0.2;  // Example position to close claws

    @Override
    public void runOpMode() {
        initRobotHardware();
        initFoodClaws();
        initAprilTagDetection();

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            controlRobot();
            controlFoodClaws(); // Add control for FoodClaws
            detectAprilTags(); // Add AprilTag detection

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }
    }

    private void initRobotHardware() {
        rightDrive = hardwareMap.get(DcMotorEx.class, "RightMotors");
        leftDrive = hardwareMap.get(DcMotorEx.class, "LeftMotors");
        lifter1 = hardwareMap.get(DcMotorEx.class, "LifterMotors1");
        StorageDoor1 = hardwareMap.get(CRServo.class, "StorageDoor1");
        DownStorageDoor1 = hardwareMap.get(CRServo.class, "DownStorageDoor1");
        DownStorageDoor2 = hardwareMap.get(CRServo.class, "DownStorageDoor1");

        // Initialize the new Antena motor
        antena = hardwareMap.get(DcMotorEx.class, "Antena");

        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        lifter1.setDirection(DcMotorEx.Direction.REVERSE);

        antena.setDirection(DcMotorEx.Direction.FORWARD); // Set the direction as needed
    }

    private void initFoodClaws() {
        // Initialize servos for food claws
        foodClawServo1 = hardwareMap.get(Servo.class, "FoodClawServo1");
        foodClawServo2 = hardwareMap.get(Servo.class, "FoodClawServo2");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        // Set initial positions for the servos
        foodClawServo1.setPosition(CLAW_CLOSED_POSITION);
        foodClawServo2.setPosition(CLAW_CLOSED_POSITION);
    }

    private void initAprilTagDetection() {
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
    }

    private void controlRobot() {
        double RightPower;
        double LeftPower;

        // Button for x movement and y movement
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        RightPower = Range.clip(drive + turn, -1, 1);
        LeftPower = Range.clip(drive - turn, -1, 1);

        // Button for Booster
        if (gamepad1.right_trigger > 0) {
            speed = 0.5;
        } else {
            speed = 1;
        }
        if (gamepad1.dpad_down){
            DownStorageDoor1.setPower(-1);
            DownStorageDoor2.setPower(-1);
        }
        if (gamepad1.dpad_down){
            DownStorageDoor1.setPower(1);
            DownStorageDoor2.setPower(1);
        }

        // Intake control
        if (gamepad1.y) {
            lifter1.setPower(1);
        } else if (gamepad1.a) {
            lifter1.setPower(-1);
        } else {
            lifter1.setPower(0);
        }

        // Storage door control
        if (gamepad1.a) {
            StorageDoor1.setPower(1);

        } else if (gamepad1.b) {
            StorageDoor1.setPower(-1);

        } else {
            StorageDoor1.setPower(0);

        }

        // Control the Antena motor
        if (gamepad2.right_bumper) {
            antena.setPower(1); // Activate Antena motor
        } else {
            antena.setPower(0); // Deactivate Antena motor
        }

        if (gamepad2.left_bumper) {
            antena.setPower(-1); // Activate Antena motor
        } else {
            antena.setPower(0); // Deactivate Antena motor
        }

        leftDrive.setPower(LeftPower * speed);
        rightDrive.setPower(RightPower * speed);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "kanan (%.2f), kiri (%.2f)", RightPower, LeftPower);
        telemetry.addData("Antena", "Power: %.2f", antena.getPower());
    }

    private void controlFoodClaws() {
        // Read color sensor values
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        boolean isOrange = (red > 200 && green > 100 && blue < 100);

        if (isOrange) {
            if (!isClawOpen) {
                // Open claws to a defined position
                foodClawServo1.setPosition(CLAW_OPEN_POSITION);
                foodClawServo2.setPosition(CLAW_OPEN_POSITION);
                isClawOpen = true;
            }
        } else {
            if (isClawOpen) {
                // Close claws
                foodClawServo1.setPosition(CLAW_CLOSED_POSITION);
                foodClawServo2.setPosition(CLAW_CLOSED_POSITION);
                isClawOpen = false;
            }
        }

        telemetry.addData("Color Sensor", "Red: %d, Green: %d, Blue: %d", red, green, blue);
    }

    private void detectAprilTags() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        if (!detections.isEmpty()) {
            // Telemetry update for tag detected
            telemetry.addLine("+-------------------------+");
            telemetry.addLine("|   ____    _  __        |");
            telemetry.addLine("|  / __ \\  | |/ /        |");
            telemetry.addLine("| | |  | | | ' /         |");
            telemetry.addLine("| | |  | | |  <          |");
            telemetry.addLine("| | |__| | | . \\         |");
            telemetry.addLine("|  \\____/  |_|\\_\\        |");
            telemetry.addLine("+-------------------------+");

            telemetry.addLine("Tag Detected:");
            for (AprilTagDetection detection : detections) {
                telemetry.addData("ID", detection.id);
            }

            gamepad1.setLedColor(0, 1, 0, 200); // Green LED for successful detection
        } else {
            gamepad1.setLedColor(1, 0, 0, 200); // Red LED if no tag detected
        }
    }
}
