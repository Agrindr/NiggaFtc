package org.firstinspires.ftc.robotcontroller;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Athenacodetest1 extends LinearOpMode {

    // Robot Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightDrive = null;
    private DcMotorEx leftDrive = null;
    private DcMotorEx lifter1 = null;
    private CRServo StorageDoor1 = null;
    private CRServo DownStorageDoor1 = null;
    private CRServo DownStorageDoor2 = null;

    private Servo FoodClaw1 = null; // Servo for FoodClaw1
    private Servo FoodClaw2 = null; // Servo for FoodClaw2
    private DcMotorEx antena = null;    // New Core Hex Motor for Antena
    private ColorSensor syauqiahhsensor = null; // Color Sensor for detecting orange

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open
    private boolean isAutonomousActive = false; // Flag to check if autonomous mode is active

    // AprilTag Detection Variables
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // Odometry Variables
    private DcMotorEx xOdometry, yOdometry;
    private double robotX = 0, robotY = 0;

    private static final double TICKS_PER_CM = 10; // Adjust this value according to your robot
    private static final double CLAW_OPEN_POSITION = 0.6; // Servo position for claws open (adjust as needed)
    private static final double CLAW_CLOSE_POSITION = 0.0; // Servo position for claws closed (adjust as needed)

    @Override
    public void runOpMode() {
        initRobotHardware();
        initFoodClaws();
        initAprilTagDetection();
        initOdometry();

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            if (isAutonomousActive) {
                // Run autonomous sequence for 10 seconds
                long startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 10000 && opModeIsActive()) {
                    runAutonomousSequence();
                    telemetry.update();
                }
                isAutonomousActive = false;
            } else {
                controlRobot();
                controlFoodClaws();
                detectAprilTags();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
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
        FoodClaw1 = hardwareMap.get(Servo.class, "FoodClaw1");
        FoodClaw2 = hardwareMap.get(Servo.class, "FoodClaw2");
        syauqiahhsensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        // Initialize the FoodClaws to the closed position
        FoodClaw1.setPosition(CLAW_CLOSE_POSITION);
        FoodClaw2.setPosition(CLAW_CLOSE_POSITION);
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

    private void initOdometry() {
        xOdometry = hardwareMap.get(DcMotorEx.class, "xOdometry");
        yOdometry = hardwareMap.get(DcMotorEx.class, "yOdometry");

        // Reset encoders
        resetOdometryEncoders();
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
        int red = syauqiahhsensor.red();
        int green = syauqiahhsensor.green();
        int blue = syauqiahhsensor.blue();

        boolean isOrange = (red > 200 && green > 100 && blue < 100);

        if (isOrange) {
            if (!isClawOpen) {
                // Open claws to the specified position
                FoodClaw1.setPosition(CLAW_OPEN_POSITION);
                FoodClaw2.setPosition(CLAW_OPEN_POSITION);
                isClawOpen = true;
            }
        } else {
            if (isClawOpen) {
                // Close claws to the specified position
                FoodClaw1.setPosition(CLAW_CLOSE_POSITION);
                FoodClaw2.setPosition(CLAW_CLOSE_POSITION);
                isClawOpen = false;
            }
        }
    }

    private void detectAprilTags() {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        if (currentDetections.size() != 0) {
            telemetry.addLine("Detected tag(s):");
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            }





        } else {
            telemetry.addLine("No tags detected");
        }
    }

    private void runAutonomousSequence() {
        moveForward(100); // Move forward 100 cm
        moveBackward(100); // Move backward 100 cm
        moveLeft(100); // Move left 100 cm
        moveForward(50); // Move forward 50 cm
        moveBackward(50); // Move backward 50 cm
        moveRight(50); // Move right 50 cm
        moveForward(100); // Move forward 100 cm
    }

    private void moveForward(double distanceCm) {
        int targetPosition = (int) (distanceCm * TICKS_PER_CM);
        xOdometry.setTargetPosition(targetPosition);
        xOdometry.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        xOdometry.setPower(0.5);

        while (xOdometry.isBusy() && opModeIsActive()) {
            telemetry.addData("Autonomous", "Moving forward: %d cm", distanceCm);
            telemetry.update();
        }
    }

    private void moveBackward(double distanceCm) {
        moveForward(-distanceCm);
    }

    private void moveLeft(double distanceCm) {
        int targetPosition = (int) (distanceCm * TICKS_PER_CM);
        yOdometry.setTargetPosition(-targetPosition);
        yOdometry.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        yOdometry.setPower(0.5);

        while (yOdometry.isBusy() && opModeIsActive()) {
            telemetry.addData("Autonomous", "Moving left: %d cm", distanceCm);
            telemetry.update();
        }
    }

    private void moveRight(double distanceCm) {
        moveLeft(-distanceCm);
    }

    private void resetOdometryEncoders() {
        xOdometry.setMode(RunMode.STOP_AND_RESET_ENCODER);
        yOdometry.setMode(RunMode.STOP_AND_RESET_ENCODER);

        xOdometry.setMode(RunMode.RUN_USING_ENCODER);
        yOdometry.setMode(RunMode.RUN_USING_ENCODER);
    }
}
