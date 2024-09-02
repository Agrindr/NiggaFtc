package org.firstinspires.ftc.robotcontroller;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "RobotControl with AprilTag Detection", group = "Concept")
public class MergedOpMode extends LinearOpMode {

    // Robot Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightDrive = null;
    private DcMotorEx leftDrive = null;
    private DcMotorEx lifter1 = null;
    private CRServo StorageDoor1 = null;
    private CRServo DownStorageDoor1 = null;
    private CRServo DownStorageDoor2 = null;

    private DcMotorEx FoodClaw1 = null; // Core Hex Motor for FoodClaw1
    private DcMotorEx FoodClaw2 = null; // Core Hex Motor for FoodClaw2
    private DcMotorEx antena = null;    // New Core Hex Motor for Antena
    private ColorSensor colorSensor = null; // Color Sensor for detecting orange

    // PID control variables for Core Hex Motors
    private static final double PID_P = 1.0; // Proportional gain
    private static final double PID_I = 0.0; // Integral gain
    private static final double PID_D = 0.0; // Derivative gain
    private static final double DEGREE_60_TICKS = 1000; // Ticks for 60 degrees, adjust as needed
    private static final double TICKS_PER_CM = 10; // Adjust this value according to your robot

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open
    private boolean isAutonomousActive = false; // Flag to check if autonomous mode is active

    // AprilTag Detection Variables
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

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

            // Start autonomous driving if AprilTag detected and Y button is pressed
            if (isAutonomousActive && gamepad1.y) {
                runAutonomousDrive(500); // Drive forward 500 cm
            }

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
        DownStorageDoor2 = hardwareMap.get(CRServo.class, "DownStorageDoor2");

        // Initialize the new Antena motor
        antena = hardwareMap.get(DcMotorEx.class, "Antena");

        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        lifter1.setDirection(DcMotorEx.Direction.REVERSE);

        antena.setDirection(DcMotorEx.Direction.FORWARD); // Set the direction as needed
    }

    private void initFoodClaws() {
        FoodClaw1 = hardwareMap.get(DcMotorEx.class, "FoodClaw1");
        FoodClaw2 = hardwareMap.get(DcMotorEx.class, "FoodClaw2");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        FoodClaw1.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FoodClaw1.setMode(RunMode.RUN_USING_ENCODER);
        FoodClaw2.setMode(RunMode.STOP_AND_RESET_ENCODER);
        FoodClaw2.setMode(RunMode.RUN_USING_ENCODER);
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

        // Control Down Storage Door
        if (gamepad1.dpad_down) {
            DownStorageDoor1.setPower(-1);
            DownStorageDoor2.setPower(-1);
        } else if (gamepad1.dpad_up) {
            DownStorageDoor1.setPower(1);
            DownStorageDoor2.setPower(1);
        } else {
            DownStorageDoor1.setPower(0);
            DownStorageDoor2.setPower(0);
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
        } else if (gamepad2.left_bumper) {
            antena.setPower(-1); // Activate Antena motor in reverse
        } else {
            antena.setPower(0); // Deactivate Antena motor
        }

        leftDrive.setPower(LeftPower * speed);
        rightDrive.setPower(RightPower * speed);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Right (%.2f), Left (%.2f)", RightPower, LeftPower);
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
                // Open claws to 60 degrees
                setClawPosition(DEGREE_60_TICKS);
                isClawOpen = true;
            }
        } else {
            if (isClawOpen) {
                // Close claws
                setClawPosition(0);
                isClawOpen = false;
            }
        }

        telemetry.addData("Color Sensor", "Red: %d, Green: %d, Blue: %d", red, green, blue);
        telemetry.addData("Claw Status", isClawOpen ? "Open" : "Closed");
    }

    private void setClawPosition(double targetPosition) {
        // PID control for Core Hex motors to achieve the desired position
        double error1 = targetPosition - FoodClaw1.getCurrentPosition();
        double error2 = targetPosition - FoodClaw2.getCurrentPosition();

        double motorPower1 = PID_P * error1;
        double motorPower2 = PID_P * error2;

        FoodClaw1.setPower(Range.clip(motorPower1, -1.0, 1.0));
        FoodClaw2.setPower(Range.clip(motorPower2, -1.0, 1.0));
    }

    private void detectAprilTags() {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        if (currentDetections.size() > 0) {
            telemetry.addLine("Detected tag ID's:");
            for (AprilTagDetection tag : currentDetections) {
                telemetry.addLine(String.format("\t%d", tag.id));
                isAutonomousActive = true; // Set autonomous mode to active if tag detected
                gamepad1.setLedColor(0, 1, 0, 200); // Set controller LED to green
            }
        } else {
            telemetry.addLine("No tag detected");
            isAutonomousActive = false; // Reset autonomous mode if no tag detected
        }
    }

    private void runAutonomousDrive(double distanceCm) {
        double ticks = distanceCm * TICKS_PER_CM;

        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) ticks);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) ticks);

        rightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(1.0);
        leftDrive.setPower(1.0);

        while (rightDrive.isBusy() && leftDrive.isBusy()) {
            // Wait for motors to reach the target
        }

        rightDrive.setPower(0);
        leftDrive.setPower(0);

        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
