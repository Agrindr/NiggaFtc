package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class
RobotPrototype extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx RightMotors = null;
    private DcMotorEx LeftMotors = null;

    private DcMotorEx LifterMotors2 = null;
    private Servo StorageDoor1 = null;
    private Servo StorageDoor2 = null;
    double speed = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Color (G)", "Init");
        telemetry.addData("Color (R)", "Init");
        telemetry.addData("Color (B)", "Init");
        telemetry.addData("Jarak", "Init");
        telemetry.update();

        RightMotors = hardwareMap.get(DcMotorEx.class, "RightMotors");
        LeftMotors = hardwareMap.get(DcMotorEx.class, "LeftMotors");

        LifterMotors2 = hardwareMap.get(DcMotorEx.class, "LifterMotors2");
        StorageDoor1 = hardwareMap.get(Servo.class, "StorageDoor1");
        StorageDoor2 = hardwareMap.get(Servo.class, "StorageDoor2");

        RightMotors.setDirection(DcMotorEx.Direction.FORWARD);
        LeftMotors.setDirection(DcMotorEx.Direction.REVERSE);

        LifterMotors2.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addData("position", 0);
        telemetry.addData("power", "Init");
        telemetry.addData("Error", "Error");
        telemetry.addData("Setpoint", 100);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.update();

            double RightPower;
            double LeftPower;

            //Button for x movement and y movement
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double Lifter = gamepad1.right_stick_y;
            RightPower = Range.clip(drive + turn, -1, 1);
            LeftPower = Range.clip(drive - turn, -1, 1);

            //Button for Booster
            if (gamepad1.right_trigger > 0){
                speed = 0.5;
            } else {
                speed = 1;
            }

            // intake
            LifterMotors2.setPower(Lifter);



            if (gamepad1.a){
                StorageDoor1.setPosition(1.0); // Set to maximum position
                StorageDoor2.setPosition(1.0);
            } else if (gamepad1.b){
                StorageDoor1.setPosition(0.0); // Set to minimum position
                StorageDoor2.setPosition(0.0);
            }


            LeftMotors.setPower(LeftPower);
            RightMotors.setPower(RightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "kanan (%.2f), kiri (%.2f)", RightPower, LeftPower);
            telemetry.update();


        }
    }
}
