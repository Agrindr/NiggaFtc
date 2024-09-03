package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private CRServo storageDoor2 = null;
    private CRServo downStorageDoor1 = null;
    private CRServo downStorageDoor2 = null;
    private DcMotorEx lifter2 = null;
    private CRServo foodClawServo1 = null; // CRServo for FoodClaw1
    private CRServo foodClawServo2 = null; // CRServo for FoodClaw2
    private DcMotorEx antena = null;    // New Core Hex Motor for Antena

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open

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
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftMotor");
        lifter1 = hardwareMap.get(DcMotorEx.class, "lifter1");
        storageDoor1 = hardwareMap.get(CRServo.class, "StorageDoor1");
        storageDoor2 = hardwareMap.get(CRServo.class, "StorageDoor2");
        downStorageDoor1 = hardwareMap.get(CRServo.class, "downStorageDoor1");
        downStorageDoor2 = hardwareMap.get(CRServo.class, "downStorageDoor2");
        lifter2 = hardwareMap.get(DcMotorEx.class, "lifter2");
        // Initialize the new Antena motor
        antena = hardwareMap.get(DcMotorEx.class, "Antena");

        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        lifter1.setDirection(DcMotorEx.Direction.REVERSE);

        antena.setDirection(DcMotorEx.Direction.FORWARD); // Set the direction as needed
    }

    private void initFoodClaws() {
        // Initialize CRServos for food claws
        foodClawServo1 = hardwareMap.get(CRServo.class, "FoodClawServo1");
        foodClawServo2 = hardwareMap.get(CRServo.class, "FoodClawServo2");

        // Stop CRServos initially
        foodClawServo1.setPower(0);
        foodClawServo2.setPower(0);
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
            lifter2.setPower(1);
        } else if (gamepad1.a) {
            lifter1.setPower(-1);
            lifter2.setPower(-1);
        } else {
            lifter1.setPower(0);
            lifter2.setPower(0);
        }

        // Storage door control
        if (gamepad1.a) {
            storageDoor1.setPower(1);
            storageDoor2.setPower(1);
        } else if (gamepad1.b) {
            storageDoor1.setPower(-1);
            storageDoor2.setPower(-1);
        } else {
            storageDoor1.setPower(0);
            storageDoor2.setPower(0);
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
                // Open claws by setting power
                foodClawServo1.setPower(1);
                foodClawServo2.setPower(1);
                isClawOpen = true;
            }
        } else if (gamepad1.b) {
            if (isClawOpen) {
                // Close claws by setting power in reverse
                foodClawServo1.setPower(-1);
                foodClawServo2.setPower(-1);
                isClawOpen = false;
            }
        } else {
            // Stop the servos when not pressing the buttons
            foodClawServo1.setPower(0);
            foodClawServo2.setPower(0);
        }

        telemetry.addData("Claw Status", isClawOpen ? "Open" : "Closed");
    }
}
