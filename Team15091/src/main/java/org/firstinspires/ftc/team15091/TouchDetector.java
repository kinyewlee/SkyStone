package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchDetector implements IObjectDetector {
    private TouchSensor touchSensor;

    TouchDetector(TouchSensor touchSensor) {
        this.touchSensor = touchSensor;
    }

    @Override
    public boolean objectDetected() {
        return touchSensor.isPressed();
    }

    @Override
    public void reset() {

    }

    @Override
    public void dispose() {

    }
}
