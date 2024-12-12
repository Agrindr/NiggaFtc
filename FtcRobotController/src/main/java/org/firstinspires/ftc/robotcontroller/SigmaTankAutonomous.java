package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SigmaTankAutonomous", group = "Sigma")
public class SigmaTankAutonomous extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Set motor directions
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            // Move forward for 1 second
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            sleep(1000);

            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
            sleep(500);

            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
            sleep(500);

            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            sleep(200);

            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
            sleep(500);

            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
            sleep(500);

            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
            sleep(500);

            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
            sleep(500);

            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
            sleep(500);

            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
            sleep(500);

            leftMotor.setPower(0.8);
            rightMotor.setPower(0.67);
            sleep(500);





            // Stop the motors
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
}
