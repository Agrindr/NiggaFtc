package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AgiSigma Autonomous with XY Odometry", group = "Linear Opmode")
public class AgiSigmaAutonomous extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor encoderX;
    private DcMotor encoderY;

    // Odometry variables
    private double x = -36.90; // Starting x position
    private double y = 37.27;  // Starting y position
    private double heading = 0.0; // Starting heading in radians

    // Constants for odometry
    private static final double TICKS_PER_REV = 1120;  // Example value for encoder counts per revolution
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Wait for the start button to be pressed
        waitForStart();

        // Reset odometry
        resetOdometry();

        if (opModeIsActive()) {
            // Starting Pose (-36.90, 37.27, 0.00 degrees)
            setPoseEstimate(x, y, heading);

            // Execute movement sequence
            driveToPoint(-15.57, 36.52);
            driveToPoint(11.61, 37.27);
            driveToPoint(28.21, 37.84);
            driveToPoint(28.40, 13.87);
            driveToPoint(25.57, -5.94);
            driveToPoint(31.23, -13.87);
            driveToPoint(28.21, -28.59);
            driveToPoint(27.46, -40.48);
            driveToPoint(-5.19, -49.35);
            driveToPoint(-24.06, -44.07);
            driveToPoint(-32.37, -25.01);
            driveToPoint(-28.97, 1.23);
            driveToPoint(-28.59, 20.48);
            driveToPoint(-41.24, 33.88);
            driveToPoint(-39.35, 40.29);
            driveToPoint(-29.91, 37.46);
            driveToPoint(-36.14, 35.01);

            // Drive back to the starting position
            driveToPoint(-36.90, 37.27);

            // Stop all motors at the end of the routine
            stopMotors();
        }
    }

    private void initHardware() {
        // Initialize the motors from the hardware map
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Initialize the additional motors for X and Y encoders
        encoderX = hardwareMap.get(DcMotor.class, "xEncoder");
        encoderY= hardwareMap.get(DcMotor.class, "yEncoder");

        // Set all motors to zero power
        setMotorPowers(0, 0);

        // Set motor modes for encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetOdometry() {
        // Reset the odometry to initial values
        x = -36.90;
        y = 37.27;
        heading = 0.0;

        // Reset motor encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void updateOdometry() {
        // Get encoder ticks from x and y encoders
        int xTicks = encoderX.getCurrentPosition();
        int yTicks = encoderY.getCurrentPosition();

        // Convert ticks to inches
        double deltaX = xTicks / TICKS_PER_INCH;
        double deltaY = yTicks / TICKS_PER_INCH;

        // Calculate the change in heading using the left and right motor encoders
        int leftTicks = leftMotor.getCurrentPosition();
        int rightTicks = rightMotor.getCurrentPosition();

        double leftDistance = leftTicks / TICKS_PER_INCH;
        double rightDistance = rightTicks / TICKS_PER_INCH;

        // Update heading
        double deltaHeading = (rightDistance - leftDistance) / WHEEL_DIAMETER_INCHES;
        heading += deltaHeading;

        // Update X and Y positions
        x += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        y += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

        // Reset motor encoders after updating odometry
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update telemetry with new pose estimate
        setPoseEstimate(x, y, heading);
    }

    private void setPoseEstimate(double x, double y, double heading) {
        // Set the initial pose estimate
        telemetry.addData("Pose Estimate", "X: %.2f, Y: %.2f, Heading: %.2f", x, y, Math.toDegrees(heading));
        telemetry.update();
    }

    private void driveToPoint(double targetX, double targetY) {
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        double targetHeading = Math.atan2(deltaY, deltaX);
        double distance = Math.hypot(deltaX, deltaY);

        // Turn to the target heading
        turnToAngle(targetHeading - heading);

        // Drive forward to the point
        driveForward(distance);

        // Update odometry after movement
        updateOdometry();
    }

    private void turnToAngle(double angle) {
        // Turn the robot to the specified angle
        telemetry.addData("Turning", "To angle: %.2f degrees", Math.toDegrees(angle));
        telemetry.update();

        // Example: Simple turn logic for two motors
        if (angle > 0) {
            // Turn right
            setMotorPowers(0.5, -0.5); // Right motor moves backward, left motor moves forward
        } else {
            // Turn left
            setMotorPowers(-0.5, 0.5); // Left motor moves backward, right motor moves forward
        }
        sleep(500); // Adjust this delay based on actual turn speed and angle
        setMotorPowers(0, 0);
    }

    private void driveForward(double distance) {
        // Drive the robot forward for a certain distance
        telemetry.addData("Driving", "Forward distance: %.2f", distance);
        telemetry.update();

        // Example: Simple drive forward logic
        setMotorPowers(0.5, 0.5); // Drive both motors forward
        sleep(1000); // Adjust this delay based on actual speed and distance
        setMotorPowers(0, 0);
    }

    private void stopMotors() {
        // Stop all motors
        setMotorPowers(0, 0);
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        // Set motor powers for left and right motors
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
