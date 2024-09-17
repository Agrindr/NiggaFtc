package org.firstinspires.ftc.teamcode.SubsystemTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;

@Config
@TeleOp(name = "Lifter PID Tuner", group = "PID Tuning")
public class LifterPIDTuner extends LinearOpMode {
    PIDCoefficients pidCoefficients = new PIDCoefficients(0.1, 0, 0);
    PID lifterPID = new PID(pidCoefficients);

    DcMotorEx lifterMotor1;
    DcMotorEx lifterMotor2;

    enum LifterState {
        max,
        min
    }
    LifterState lifterState = LifterState.max;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        lifterMotor1 = hardwareMap.get(DcMotorEx.class, "LifterMotor1");
        lifterMotor2 = hardwareMap.get(DcMotorEx.class, "LifterMotor2");

        lifterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        lifterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        updateSetpoint();
        waitForStart();
        while (opModeIsActive()) {
            updateSetpoint();
            if (gamepad1.x) {
                lifterState = LifterState.max;
            }else if (gamepad1.b) {
                lifterState = LifterState.min;
            }
            double lifterPower = lifterPID.update(lifterMotor1.getCurrentPosition());
            lifterMotor1.setPower(lifterPower);
            lifterMotor2.setPower(lifterPower);

            dashboardTelemetry.addData("Error ", lifterPID.getError());
            dashboardTelemetry.addData("Setpoint ", lifterPID.getSetpoint());
            dashboardTelemetry.addData("Position ", lifterMotor1.getCurrentPosition());


        }
    }
    public void updateSetpoint() {
        double setpoint = 0;
        if (lifterState == LifterState.max) {
            setpoint = 1000;
            if (lifterPID.getSetpoint() != setpoint) {
                lifterPID.setSetpoint(setpoint);
            }
        }else {
            setpoint = 0;
            if (lifterPID.getSetpoint() != setpoint) {
                lifterPID.setSetpoint(setpoint);
            }
        }
    }
}
