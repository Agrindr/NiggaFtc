package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SimplifiedOpMode extends LinearOpMode {

    // Robot Control Variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightDrive = null;
    private DcMotorEx leftDrive = null;
    private DcMotorEx lifter1 = null;
    private CRServo storageDoor1 = null;
    private CRServo downStorageDoor1 = null;
    private CRServo downStorageDoor2 = null;

    private Servo foodClawServo1 = null; // Servo for FoodClaw1
    private Servo foodClawServo2 = null; // Servo for FoodClaw2
    private DcMotorEx antena = null;    // New Core Hex Motor for Antena

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open

    // Servo position constants for food claws
    private static final double CLAW_OPEN_POSITION = 0.6;  // Example position to open claws
    private static final double CLAW_CLOSED_POSITION = 0.2;  // Example position to close claws

    @Override
    public void runOpMode() {
        initRobotHardware();
        initFoodClaws();

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            controlRobot();
            controlFoodClaws(); // Add control for FoodClaws

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
        storageDoor1 = hardwareMap.get(CRServo.class, "StorageDoor1");
        downStorageDoor1 = hardwareMap.get(CRServo.class, "DownStorageDoor1");
        downStorageDoor2 = hardwareMap.get(CRServo.class, "DownStorageDoor2");

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

        // Set initial positions for the servos
        foodClawServo1.setPosition(CLAW_CLOSED_POSITION);
        foodClawServo2.setPosition(CLAW_CLOSED_POSITION);
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

        if (gamepad1.dpad_down) {
            downStorageDoor1.setPower(-1);
            downStorageDoor2.setPower(-1);
        } else if (gamepad1.dpad_up) {
            downStorageDoor1.setPower(1);
            downStorageDoor2.setPower(1);
        } else {
            downStorageDoor1.setPower(0);
            downStorageDoor2.setPower(0);
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
            storageDoor1.setPower(1);
        } else if (gamepad1.b) {
            storageDoor1.setPower(-1);
        } else {
            storageDoor1.setPower(0);
        }

        // Control the Antena motor
        if (gamepad2.right_bumper) {
            antena.setPower(1); // Activate Antena motor
        } else if (gamepad2.left_bumper) {
            antena.setPower(-1); // Activate Antena motor
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
        if (gamepad1.x) {
            if (!isClawOpen) {
                // Open claws to a defined position
                foodClawServo1.setPosition(CLAW_OPEN_POSITION);
                foodClawServo2.setPosition(CLAW_OPEN_POSITION);
                isClawOpen = true;
            }
        } else if (gamepad1.b) {
            if (isClawOpen) {
                // Close claws
                foodClawServo1.setPosition(CLAW_CLOSED_POSITION);
                foodClawServo2.setPosition(CLAW_CLOSED_POSITION);
                isClawOpen = false;
            }
        }

        telemetry.addData("Claw Status", isClawOpen ? "Open" : "Closed");
    }
}
