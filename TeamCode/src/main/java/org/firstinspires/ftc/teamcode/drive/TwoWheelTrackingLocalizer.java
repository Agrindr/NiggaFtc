

package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public static double TICKS_PER_REV = 288 ;
    public static double WHEEL_RADIUS = 2; // in

    public static double PERPENDICULAR_WHEEL_RADIUS = 2.3622 / 2; // in
    public static double PARALLEL_WHEEL_RADIUS = 2.3622 / 2; // in

    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_GEAR_RATIO = 1/15; // or 15/20
    public static double PARALLEL_SPROCKET_RATIO = 1/(2 * 3 * 72/15); // 15 -> 30 | 15 -> 45 | 15 -> 72
    public static double PERPENDICULAR_SPROCKET_RATIO = 1/(5 * 5 * 3/2);

    public static double PARALLEL_X = 18; // X is the up and down direction
    public static double PARALLEL_Y = -5; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleTankDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleTankDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "xEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "yEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static double parallelTicksToInches(double ticks) {
        return PARALLEL_WHEEL_RADIUS * 2 * Math.PI  * PARALLEL_SPROCKET_RATIO * ticks / TICKS_PER_REV;
    }

    // the additional hd hex motor
    public static double perpendicularTicksToInches(double ticks) {
        return PERPENDICULAR_WHEEL_RADIUS * 2 * Math.PI * PERPENDICULAR_SPROCKET_RATIO * ticks / TICKS_PER_REV;
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
        return Arrays.asList(
                parallelTicksToInches(parallelEncoder.getCurrentPosition()),
                perpendicularTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                parallelTicksToInches(parallelEncoder.getRawVelocity()),
                perpendicularTicksToInches(perpendicularEncoder.getRawVelocity())
        );
    }
}
