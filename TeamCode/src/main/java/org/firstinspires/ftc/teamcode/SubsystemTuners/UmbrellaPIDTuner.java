package org.firstinspires.ftc.teamcode.SubsystemTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;

@Config
@TeleOp(name = "Umbrella PID Tuner", group = "PID Tuning")
public class UmbrellaPIDTuner extends LinearOpMode {
    PIDCoefficients pidCoefficients = new PIDCoefficients(0.1, 0, 0);
    PID umbrellaPID = new PID(pidCoefficients);

    private DcMotorEx UmbrellaMotor;

    enum LifterState {
        max,
        min
    }
    LifterPIDTuner.LifterState lifterState = LifterPIDTuner.LifterState.max;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        UmbrellaMotor = hardwareMap.get(DcMotorEx.class, "UmbrellaMotor");


    }
}
