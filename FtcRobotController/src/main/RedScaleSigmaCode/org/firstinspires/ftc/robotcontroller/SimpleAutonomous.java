package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Simple Autonomous with 2 Motors", group = "Linear Opmode")
public class SimpleAutonomous extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            // Starting Pose (-36.90, 37.27, 0.00 degrees)
            setPoseEstimate(-36.90, 37.27, 0.00);

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

            // Stop all motors at the end of the routine
            stopMotors();
        }
    }

    private void initHardware() {
        // Initialize the motors from the hardware map
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Set all motors to zero power
        setMotorPowers(0, 0);
    }

    private void setPoseEstimate(double x, double y, double heading) {
        // Set the initial pose estimate
        telemetry.addData("Pose Estimate", "X: %.2f, Y: %.2f, Heading: %.2f", x, y, Math.toDegrees(heading));
        telemetry.update();
    }

    private void driveToPoint(double x, double y) {
        double currentX = 0; // Assume starting at the origin for simplicity
        double currentY = 0; // Assume starting at the origin for simplicity
        double currentHeading = 0; // Assume facing right initially

        // Calculate the distance and angle to the target point
        double distance = Math.hypot(x - currentX, y - currentY);
        double targetAngle = Math.atan2(y - currentY, x - currentX);
        double turnAngle = targetAngle - currentHeading;

        // Turn to the correct angle
        turnToAngle(turnAngle);

        // Drive forward to the point
        driveForward(distance);

        // Update telemetry with new pose estimate
        setPoseEstimate(x, y, targetAngle);
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
