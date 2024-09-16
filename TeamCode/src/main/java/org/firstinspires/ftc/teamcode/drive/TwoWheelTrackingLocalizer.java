

package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    private Telemetry telemetry;

    public static double TICKS_PER_REV = 288 ;
    public static double WHEEL_RADIUS = 2; // in

    public static double PERPENDICULAR_WHEEL_RADIUS = CM2INCH(6) / 2; // in
    public static double PARALLEL_WHEEL_RADIUS = CM2INCH(6) / 2; // in

    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_SPROCKET_RATIO = (double) 1 /(3 * 2 * 4); // X Axis 15 -> 45 | 30 -> 60 | 15 -> 60
    public static double PERPENDICULAR_SPROCKET_RATIO = (double) 1 /((double) (5 * 5 * 3) /2); // Y Axis

    public static double PARALLEL_X = CM2INCH(-8.5); // X is the up and down direction
    public static double PARALLEL_Y = CM2INCH(17); // Y is the strafe direction

    public static double PERPENDICULAR_X = CM2INCH(8.5);
    public static double PERPENDICULAR_Y = CM2INCH(13);

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;
    private DcMotorEx ParallelEncoder, PerpendicularEncoder;

    private SampleTankDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleTankDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "encoderX"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "encoderY"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleTankDrive drive, Telemetry telemetry) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        this.telemetry = telemetry;

        ParallelEncoder = hardwareMap.get(DcMotorEx.class, "encoderX");
        PerpendicularEncoder = hardwareMap.get(DcMotorEx.class, "encoderY");
        ParallelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        PerpendicularEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        ParallelEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        PerpendicularEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public  double parallelTicksToInches(double ticks) {
        return ticks * ((PARALLEL_WHEEL_RADIUS * 2 * Math.PI) / (TICKS_PER_REV * PARALLEL_SPROCKET_RATIO));
    }

    // the additional hd hex motor
    public  double perpendicularTicksToInches(double ticks) {
        return  ticks * ((PERPENDICULAR_WHEEL_RADIUS * 2 * Math.PI) / (TICKS_PER_REV * PERPENDICULAR_SPROCKET_RATIO));
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        double parralelInch = parallelTicksToInches(ParallelEncoder.getCurrentPosition());
        double perpendicularInch = perpendicularTicksToInches(PerpendicularEncoder.getCurrentPosition());
        telemetry.addData("parralelInch", ParallelEncoder.getCurrentPosition() * ((PARALLEL_WHEEL_RADIUS * 2 * Math.PI) / (TICKS_PER_REV * PARALLEL_SPROCKET_RATIO)));
        telemetry.addData("parralelInch", PerpendicularEncoder.getCurrentPosition() * ((PERPENDICULAR_WHEEL_RADIUS * 2 * Math.PI) / (TICKS_PER_REV * PERPENDICULAR_SPROCKET_RATIO)));

        telemetry.addData("perpendicularInch",perpendicularInch);
        telemetry.addData("Parralel Encoder", ParallelEncoder.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder", PerpendicularEncoder.getCurrentPosition());
//        telemetry.update();

        return Arrays.asList(
                parralelInch,
                perpendicularInch
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                parallelTicksToInches(ParallelEncoder.getVelocity()),
                perpendicularTicksToInches(PerpendicularEncoder.getVelocity())
        );
    }

    public static double CM2INCH(double cm) {
        return cm / 2.54;
    }
}
