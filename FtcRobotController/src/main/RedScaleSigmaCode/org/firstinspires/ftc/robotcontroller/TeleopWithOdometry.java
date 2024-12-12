package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp
public class TeleopWithOdometry extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Odometry encoders
    private DcMotor leftEncoder = null;
    private DcMotor rightEncoder = null;
    private DcMotor horizontalEncoder = null;

    // Position and heading
    private double robotX = 0;
    private double robotY = 0;
    private double robotHeading = 0;

    // Encoder ticks per inch (adjust based on your configuration)
    private static final double TICKS_PER_INCH = 1440; // Example value, replace with your actual value

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        
        // Initialize the hardware variables
        leftDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightDrive = hardwareMap.get(DcMotor.class, "motor2");

        leftEncoder = hardwareMap.get(DcMotor.class, "left_encoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "right_encoder");
        horizontalEncoder = hardwareMap.get(DcMotor.class, "horizontal_encoder");

        // Reverse direction of motors if necessary
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set encoders to run without encoders (if you're only using them for odometry)
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
        runtime.reset();

        // Variables to store previous encoder positions
        int previousLeftEncoderPos = 0;
        int previousRightEncoderPos = 0;
        int previousHorizontalEncoderPos = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Calculate encoder deltas
            int currentLeftEncoderPos = leftEncoder.getCurrentPosition();
            int currentRightEncoderPos = rightEncoder.getCurrentPosition();
            int currentHorizontalEncoderPos = horizontalEncoder.getCurrentPosition();

            int deltaLeft = currentLeftEncoderPos - previousLeftEncoderPos;
            int deltaRight = currentRightEncoderPos - previousRightEncoderPos;
            int deltaHorizontal = currentHorizontalEncoderPos - previousHorizontalEncoderPos;

            previousLeftEncoderPos = currentLeftEncoderPos;
            previousRightEncoderPos = currentRightEncoderPos;
            previousHorizontalEncoderPos = currentHorizontalEncoderPos;

            // Calculate movement
            double deltaForward = (deltaLeft + deltaRight) / 2.0 / TICKS_PER_INCH;
            double deltaStrafe = deltaHorizontal / TICKS_PER_INCH;

            // Update position (simple example, consider more advanced methods)
            robotX += deltaStrafe;
            robotY += deltaForward;

            // Update heading (if using a gyro or other sensor, you would update this too)

            // Drive control (POV Mode)
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Display telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Odometry", "X (%.2f), Y (%.2f)", robotX, robotY);
            telemetry.update();
        }
    }
}
