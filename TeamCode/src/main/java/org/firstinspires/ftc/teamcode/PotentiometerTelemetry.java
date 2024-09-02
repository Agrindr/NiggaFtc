package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class PotentiometerTelemetry extends OpMode {

    // Declare an AnalogInput variable
    private AnalogInput potentiometer;

    @Override
    public void init() {
        // Initialize the AnalogInput
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        // Set up telemetry sigma Sigma On the walls Whos the skibidest of them
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read the value from the potentiometer
        double potentiometerValue = potentiometer.getVoltage();

        // Update telemetry with the potentiometer value
        telemetry.addData("Potentiometer Value", potentiometerValue);
        telemetry.update();
    }
}
