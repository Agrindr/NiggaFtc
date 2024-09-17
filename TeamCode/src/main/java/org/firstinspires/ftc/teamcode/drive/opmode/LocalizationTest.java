package org.firstinspires.ftc.teamcode.drive.opmode;

import android.media.Image;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        String projectPath = System.getProperty("user.dir");
        File imageFilePath = new File(projectPath, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/FGC_FIELD.jpg");

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .drawImage("/dash/FGC_FIELD.jpg", 0, 0, 144, 144);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        packet.fieldOverlay()
                .drawImage("/dash/FGC_FIELD.jpg", 0, 0, 144, 144);
        while (!isStopRequested()) {
            dashboard.sendTelemetryPacket(packet);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_y
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("imagePath", imageFilePath.toString());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
