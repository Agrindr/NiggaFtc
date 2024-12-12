package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OnBotRedScale extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor BottomC1;
    private DcMotor BottomC2;
    private DcMotor TopC1;
    private DcMotor TopC2; // Correctly initialized
    private CRServo extender;
    private CRServo intakeWheel;
    private CRServo intakeClaw;

    @Override
    public void runOpMode() {
        // Initialize the motors and servos
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        BottomC1 = hardwareMap.get(DcMotor.class, "BottomC1");
        BottomC2 = hardwareMap.get(DcMotor.class, "BottomC2");
        TopC1 = hardwareMap.get(DcMotor.class, "TopC1");
        TopC2 = hardwareMap.get(DcMotor.class, "TopC2");
        extender = hardwareMap.get(CRServo.class, "extender");
        intakeWheel = hardwareMap.get(CRServo.class, "intakeWheel");
        intakeClaw = hardwareMap.get(CRServo.class, "intakeClaw");

        // Set motor direction (reverse if needed)
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        BottomC1.setDirection(DcMotor.Direction.FORWARD);
        BottomC2.setDirection(DcMotor.Direction.REVERSE);
        TopC1.setDirection(DcMotor.Direction.FORWARD);
        TopC2.setDirection(DcMotor.Direction.REVERSE);

        // Set a default speed multiplier
        double defaultSpeed = 0.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // --- Driving Controls ---
            double drive = -gamepad1.left_stick_y; // Forward/reverse
            double turn = gamepad1.right_stick_x; // Left/right

            // Apply booster with left trigger
            double speedMultiplier = gamepad1.left_trigger > 0 ? 1.0 : defaultSpeed;

            // Combine drive and turn inputs
            double leftPower = (drive + turn) * speedMultiplier;
            double rightPower = (drive - turn) * speedMultiplier;

            // Normalize power if it exceeds 1.0
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            // Set motor power
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // --- Linkage Controls ---
            if (gamepad1.right_bumper) {
                BottomC1.setPower(1);
                BottomC2.setPower(1);
                TopC1.setPower(0.6);
                TopC2.setPower(0.6);
            } else if (gamepad1.left_bumper) {
                BottomC1.setPower(-0.5);
                BottomC2.setPower(-0.5);
                TopC1.setPower(-0.7);
                TopC2.setPower(-0.7);
            } else {
                BottomC1.setPower(0);
                BottomC2.setPower(0);
                TopC1.setPower(0);
                TopC2.setPower(0);
            }

            // --- Extender Controls ---
            if (gamepad1.y) {
                extender.setPower(1);
            } else if (gamepad1.a) {
                extender.setPower(-1);
            } else {
                extender.setPower(0);
            }

            // --- Intake Controls ---
            // Intake wheel controls
            if (gamepad2.left_bumper) {
                intakeWheel.setPower(1); // Spin forward
            } else if (gamepad2.right_bumper) {
                intakeWheel.setPower(-1); // Spin backward
            } else {
                intakeWheel.setPower(0); // Stop
            }

            // Intake claw controls
            if (gamepad2.y) {
                intakeClaw.setPower(1); // Close claw
            } else if (gamepad2.a) {
                intakeClaw.setPower(-1); // Open claw
            } else {
                intakeClaw.setPower(0); // Stop
            }

            // --- Telemetry for Debugging ---
            telemetry.addData("Drive Power", drive);
            telemetry.addData("Turn Power", turn);
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("BottomC1 Power", BottomC1.getPower());
            telemetry.addData("BottomC2 Power", BottomC2.getPower());
            telemetry.addData("TopC1 Power", TopC1.getPower());
            telemetry.addData("TopC2 Power", TopC2.getPower());
            telemetry.addData("Extender Power", extender.getPower());
            telemetry.addData("Intake Wheel Power", intakeWheel.getPower());
            telemetry.addData("Intake Claw Power", intakeClaw.getPower());
            telemetry.update();
        }
    }
}
