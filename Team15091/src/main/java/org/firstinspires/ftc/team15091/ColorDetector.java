package org.firstinspires.ftc.team15091;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorDetector implements IObjectDetector {
    private ColorSensor sensorColor;

    ColorDetector(ColorSensor colorSensor) {
        sensorColor = colorSensor;
    }

    public boolean objectDetected() {
        boolean detected = false;
        if (sensorColor.blue() > 250 || sensorColor.red() > 250) {
            detected = true;
        }

        return detected;
    }

    public void dispose() {

    }
}
