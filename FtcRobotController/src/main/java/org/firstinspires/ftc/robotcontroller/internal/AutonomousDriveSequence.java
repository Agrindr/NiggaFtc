package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousDriveSequence", group="Linear Opmode")
public class AutonomousDriveSequence extends LinearOpMode {

    private DcMotorEx rightDrive = null;
    private DcMotorEx leftDrive = null;
    private DcMotorEx xOdometry = null;  // Track X movement
    private DcMotorEx yOdometry = null;  // Track Y movement

    private static final double TICKS_PER_REV = 1440; // Core Hex motor's CPR
    private static final double WHEEL_DIAMETER_CM = 9.0 * 2.54; // Convert inches to cm
    private static final double GEAR_RATIO = 3.0; // Adjust this to 5.0 for the 5:1 gear ratio
    private static final double TICKS_PER_CM = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_CM * Math.PI);

    // Track robot's pomsition holy shit Guys i dont know what to do i finished the program -Agi
    private int startX, startY;

    @Override
    public void runOpMode() {
        // Initialize hardware
        rightDrive = hardwareMap.get(DcMotorEx.class, "RightMotors");
        leftDrive = hardwareMap.get(DcMotorEx.class, "LeftMotors");
        xOdometry = hardwareMap.get(DcMotorEx.class, "xOdometry");
        yOdometry = hardwareMap.get(DcMotorEx.class, "yOdometry");

        // Set motor directions
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // Set zero power behavior (optional)
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Record starting position for odometry
        startX = xOdometry.getCurrentPosition();
        startY = yOdometry.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Drive forward 100 cm (1 meter)
            moveRobot(100, 0.5);

            // Move right 500 cm by turning 90 degrees to the right, driving forward, then turning back
            turnRobot(90, 0.5); // turn 90 degrees to the right
            moveRobot(500, 0.5); // drive forward 500 cm
            turnRobot(-90, 0.5); // turn back 90 degrees to face original direction

            // Turn right and move forward another 500 cm
            moveRobot(500, 0.5);

            // Return to the starting position (move left 500 cm and back 100 cm)
            turnRobot(180, 0.5); // turn 180 degrees to face the opposite direction
            moveRobot(500, 0.5); // move backward (same as forward in opposite direction)
            moveRobot(100, 0.5); // move backward another 100 cm

            // Check if the robot is back at the starting position
            if (hasReturnedToStart()) {
                telemetry.addData("Status", "Returned to Start");
                telemetry.addData("Sigma", "Confirmed");
            } else {
                telemetry.addData("Status", "Not at Start");
            }
            telemetry.update();
        }
    }

    private void moveRobot(double distanceCM, double speed) {
        int targetPosition = (int) (distanceCM * TICKS_PER_CM);

        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetPosition);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetPosition);

        rightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(speed);
        leftDrive.setPower(speed);

        while (rightDrive.isBusy() && leftDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving to Position", "Target: %d", targetPosition);
            telemetry.update();
        }

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        //Dude holy siht n fingerplease leat me test my fkin codde waaaaaaaaaa nigger nigger pants o
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void turnRobot(int degrees, double speed) {
        // Assuming a simple turn function, adjust based on your robot's turning characteristics
        int turnTicks = (int) (degrees * TICKS_PER_CM); // You may need to adjust this based on your robot's turning arc
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + turnTicks);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - turnTicks);
        // oh my god the Lifter team wont shut up go back playing ml dude please im begging you waaaaaa i hate niggers i hate those who smokes i execpt my father
        rightDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(speed);
        leftDrive.setPower(speed);

        while (rightDrive.isBusy() && leftDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Turning to Position", "Degrees: %d", degrees);
            telemetry.update();
        }

        rightDrive.setPower(0);
        leftDrive.setPower(0);

        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Check if robot has returned to the starting position
    private boolean hasReturnedToStart() {
        int currentX = xOdometry.getCurrentPosition();
        int currentY = yOdometry.getCurrentPosition();

        return Math.abs(currentX - startX) < 10 && Math.abs(currentY - startY) < 10; // Allow some tolerance
    }
}
