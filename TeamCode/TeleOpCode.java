package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint
        ;

@TeleOp
public class TeleOpCode extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorKanan = null;
    private DcMotorEx motorKiri = null;
    private DcMotorEx Lifter = null;
    private Servo Gate1 = null;


    double speed = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        motorKanan = hardwareMap.get(DcMotorEx.class, "kanan");
        motorKiri = hardwareMap.get(DcMotorEx.class, "kiri");
        Lifter = hardwareMap.get(DcMotorEx.class, "Lifter");
        Gate1 = hardwareMap.get(Servo.class,"Gate1");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorKanan.setDirection(DcMotor.Direction.REVERSE);
        motorKiri.setDirection(DcMotor.Direction.FORWARD);

        // Set default position

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double kananPower;
            double kiriPower;

            double drive = gamepad1.left_stick_y * speed;
            double turn = gamepad1.right_stick_x * speed;

            kananPower = Range.clip(drive - turn, -1.0, 1.0);
            kiriPower = Range.clip(drive + turn, -1.0, 1.0);

            if (gamepad1.left_trigger == 1) {
                speed = 1;
            } else {
                speed = 0.5;
            }
            if (gamepad2.y) {
                Lifter.setPower(1);
            } else if (gamepad2.a){
                Lifter.setPower(-1);
            } else {
                Lifter.setPower(0);
            }
            if (gamepad2.x){
                
            }
            


            motorKanan.setPower(kananPower);
            motorKiri.setPower(kiriPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "kanan (%.2f), kiri (%.2f)", kananPower, kiriPower);
            telemetry.update();
        }
    }
}