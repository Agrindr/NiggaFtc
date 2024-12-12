package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    private DcMotorEx cvlinkage1 = null;
    private DcMotorEx cvlinkage2 = null;
    private DcMotorEx encoderX = null;
    private DcMotorEx encoderY = null;
    private DigitalChannel magnetSensor = null; // General magnet sensor
    private DigitalChannel lifterMagnetSensor = null; // Lifter magnet sensor
    private DigitalChannel touchSensor = null;

    private double speed = 0;
    private boolean isClawOpen = false; // Track if claws are open

    // Odometry variables
    private double xPosition = 0;
    private double yPosition = 0;
    private int lastEncoderXPos = 0;
    private int lastEncoderYPos = 0;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftMotor");
        lifter1 = hardwareMap.get(DcMotorEx.class, "lifter1");vhttps://github.com/Agrindr/FGC-TEAM-INDONESIA-2024/pull/1
        storageDoor1 = hardwareMap.get(CRServo.class, "storageDoor1");
        storageDoor2 = hardwareMap.get(CRServo.class, "storageDoor2");
        downStorageDoor1 = hardwareMap.get(CRServo.class, "downStoragedoor1");
        downStorageDoor2 = hardwareMap.get(CRServo.class, "downStoragedoor2");
        lifter2 = hardwareMap.get(DcMotorEx.class, "lifter2");
        cvlinkage1 = hardwareMap.get(DcMotorEx.class, "cvlinkage1");
        cvlinkage2 = hardwareMap.get(DcMotorEx.class, "cvlinkage2");
        encoderX = hardwareMap.get(DcMotorEx.class, "encoderX");
        encoderY = hardwareMap.get(DcMotorEx.class, "encoderY");

        // Initialize the magnetic sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "liftertouch");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "magnetSensor"); // General magnet sensor
        lifterMagnetSensor = hardwareMap.get(DigitalChannel.class, "lifterMagnetSensor"); // Lifter magnet sensor

        magnetSensor.setMode(DigitalChannel.Mode.INPUT);  // Set as input
        lifterMagnetSensor.setMode(DigitalChannel.Mode.INPUT);  // Set as input
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        lifter1.setDirection(DcMotorEx.Direction.REVERSE);
        lifter2.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset odometry encoders
        lastEncoderXPos = encoderX.getCurrentPosition();
        lastEncoderYPos = encoderY.getCurrentPosition();

        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        lifter1.setDirection(DcMotorEx.Direction.FORWARD);
        lifter2.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Control logic for the robot
            double RightPower;
            double LeftPower;

            // Button for x movement and y movement
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            RightPower = Range.clip(drive + turn, -1, 1);
            LeftPower = Range.clip(drive - turn, -1, 1);

            // Speed control with booster
            if (gamepad1.left_trigger > 0) {
                speed = 1;
            } else {
                speed = 0.8;
            }

            // Magnetic sensor detection
            boolean magnetDetected = !magnetSensor.getState(); // Assuming sensor returns false when magnet is near
            boolean lifterMagnetDetected = !lifterMagnetSensor.getState(); // Assuming lifter sensor returns false when magnet is near
            boolean isTouchSensorPressed = !touchSensor.getState();

            // Lifter control logic (allowing upward motion even when lifter magnet is detected)
            double lifterPower = -gamepad2.right_stick_y;  // Invert y-axis as up is usually negative

            if (isTouchSensorPressed && lifterPower < 0) {
                // If touch sensor is pressed and lifter is trying to go down, stop it
                lifter1.setPower(0);
                lifter2.setPower(0);
            } else {
                // Otherwise, allow lifter to move
                lifter1.setPower(lifterPower);
                lifter2.setPower(lifterPower);
            }

            if (lifterMagnetDetected && lifterPower > 0) {
                // If lifter magnet is detected and lifter is trying to go down, stop it
                lifter1.setPower(0.1);
                lifter2.setPower(0.1);
            } else {
                // Otherwise, allow lifter to move
                lifter1.setPower(lifterPower);
                lifter2.setPower(lifterPower);
            }

            // Rest of the robot control code (same as before)
            if (gamepad2.left_trigger > 0) {
                downStorageDoor1.setPower(-1);
            } else if (gamepad2.left_bumper) {
                downStorageDoor1.setPower(1);
            } else {
                downStorageDoor1.setPower(0);
            }

            if (gamepad2.right_bumper) {
                downStorageDoor2.setPower(-1);
            } else if (gamepad2.right_trigger > 0) {
                downStorageDoor2.setPower(1);
            } else {
                downStorageDoor2.setPower(0);
            }

            // Storage door control
            if (gamepad2.a) {
                storageDoor1.setPower(-1);
                storageDoor2.setPower(1);
            } else if (gamepad2.y) {
                storageDoor1.setPower(1);
                storageDoor2.setPower(-1);
            } else {
                storageDoor1.setPower(0);
                storageDoor2.setPower(0);
            }

            // Control the Antena motor
            if (!magnetDetected) {
                // If the general magnet is not detected, allow the motors to move
                if (gamepad2.dpad_up) {
                    cvlinkage1.setPower(1); // Activate Antena motor
                    cvlinkage2.setPower(-1);
                } else if (gamepad2.dpad_down) {
                    cvlinkage1.setPower(-1); // Activate Antena motor
                    cvlinkage2.setPower(1);
                } else {
                    cvlinkage1.setPower(0); // Deactivate Antena motor
                    cvlinkage2.setPower(0);
                }
            } else {
                // If the general magnet is detected, stop the Antena motors
                cvlinkage1.setPower(0);
                cvlinkage2.setPower(0);
            }

            leftDrive.setPower(LeftPower * speed);
            rightDrive.setPower(RightPower * speed);

            // Odometry update
            int currentEncoderXPos = encoderX.getCurrentPosition();
            int currentEncoderYPos = encoderY.getCurrentPosition();

            int deltaX = currentEncoderXPos - lastEncoderXPos;
            int deltaY = currentEncoderYPos - lastEncoderYPos;

            xPosition += deltaX;
            yPosition += deltaY;

            lastEncoderXPos = currentEncoderXPos;
            lastEncoderYPos = currentEncoderYPos;

            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Right (%.2f), Left (%.2f)", RightPower, LeftPower);
            telemetry.addData("Lifter", "Power: %.2f", lifterPower);
            telemetry.addData("Magnet Detected", magnetDetected ? "Yes" : "No");
            telemetry.addData("Lifter Magnet Detected", lifterMagnetDetected ? "Yes" : "No");
            telemetry.addData("Touch Sensor Pressed", isTouchSensorPressed ? "Yes" : "No");
            telemetry.addData("Odometry", "X: %.2f, Y: %.2f", xPosition, yPosition);
            telemetry.update();
        }
    }
}
