    package org.firstinspires.ftc.robotcontroller;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    @Autonomous(name = "OdometryMovement")
    public class OdometryMovement extends LinearOpMode {

        // Motors for movement
        private DcMotorEx leftMotor, rightMotor;

        // Odometry wheels for tracking movement (Core Hex motors)
        private DcMotorEx xOdometry, yOdometry;

        // Constants for odometry
        private static final double COUNTS_PER_REV = 1440; // Core Hex motor's CPR
        private static final double WHEEL_DIAMETER_INCHES = 9.0;
        private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

        // Track the robot's position
        private double robotX = 0, robotY = 0;

        @Override
        public void runOpMode() {
            // Initialize motors
            leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
            rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

            // Initialize odometry wheels (Core Hex motors)
            xOdometry = hardwareMap.get(DcMotorEx.class, "xOdometry");
            yOdometry = hardwareMap.get(DcMotorEx.class, "yOdometry");

            // Reset encoders
            resetEncoders();

            waitForStart();

            // Move forward
            moveStraight(24, 0.5); // Move forward 24 inches
            // Move backward
            moveStraight(-24, 0.5); // Move backward 24 inches
            // Move left (pivot in place)
            pivotLeft(24, 0.5); // Pivot left by rotating in place
            // Move forward again
            moveStraight(24, 0.5);
            // Move backward again
            moveStraight(-24, 0.5);
            // Move right (pivot in place)
            pivotRight(24, 0.5); // Pivot right by rotating in place
            // Move forward one last time
            moveStraight(24, 0.5);
        }

        private void moveStraight(double distance, double power) {
            double targetY = robotY + distance;

            // Set motor powers for forward/backward motion
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            // Continue until target distance is reached
            while (opModeIsActive() && Math.abs(robotY - targetY) > 0.5) {
                updateOdometry();
                telemetry.addData("Target Y", targetY);
                telemetry.addData("Current Y", robotY);
                telemetry.update();
            }

            // Stop all motion
            stopAllMotors();
        }

        private void pivotLeft(double angle, double power) {
            // Set motor powers for rotating left
            leftMotor.setPower(-power);
            rightMotor.setPower(power);

            // Continue rotating for a specific time or encoder count
            sleep(1000); // Adjust timing for a 90-degree pivot

            // Stop all motion
            stopAllMotors();
        }

        private void pivotRight(double angle, double power) {
            // Set motor powers for rotating right
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

            // Continue rotating for a specific time or encoder count
            sleep(1000); // Adjust timing for a 90-degree pivot

            // Stop all motion
            stopAllMotors();
        }

        private void updateOdometry() {
            double xPosition = xOdometry.getCurrentPosition() / COUNTS_PER_INCH;
            double yPosition = yOdometry.getCurrentPosition() / COUNTS_PER_INCH;

            robotX = xPosition;
            robotY = yPosition;
        }

        private void resetEncoders() {
            xOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            yOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            xOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            yOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void stopAllMotors() {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
