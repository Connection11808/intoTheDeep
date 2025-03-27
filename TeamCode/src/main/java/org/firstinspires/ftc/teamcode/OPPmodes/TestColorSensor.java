package org.firstinspires.ftc.teamcode.OPPmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;


@TeleOp
public class TestColorSensor extends LinearOpMode {
    // Define a variable for our color sensor
    NormalizedColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            // Get normalized color values (from 0 to 1)
            NormalizedRGBA colorValues = color.getNormalizedColors();

            // Extract individual color components (red, green, blue)
            double red = colorValues.red;
            double green = colorValues.green;
            double blue = colorValues.blue;

            // Display the color values on the telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            // Detect the color based on the RGB values
            if (red + green > 0.2700 && blue < 0.6) {
                telemetry.addData("Detected Color", "Yellow");
            } else if (red > green && red > blue) {
                telemetry.addData("Detected Color", "Red");
            } else if (blue > red && blue > green) {
                telemetry.addData("Detected Color", "Blue");
            } else{
                telemetry.addData("Detected Color", "Unknown");
            }

            telemetry.update();
        }
    }
}
