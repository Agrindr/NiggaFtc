package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class LifterOnly extends OpMode {

    // Declare the motor for the lifter
    private DcMotor lifterMotor;

    @Override
    public void init() {
        // Initialize the lifter motor (assuming it's named "lifterMotor" in the configuration)
        lifterMotor = hardwareMap.get(DcMotor.class, "lifterMotor");

        // Optionally set the direction of the motor if needed
        lifterMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {

        // Control the lifter using the left joystick's Y-axis on gamepad1
        double lifterPower = -gamepad1.left_stick_y; // Invert the Y-axis if necessary

        // Set the power to the lifter motor
        lifterMotor.setPower(lifterPower);

        // Optionally, add telemetry data to monitor the lifter's status
        telemetry.addData("Lifter Power", lifterPower);
        telemetry.update();
    }
}
