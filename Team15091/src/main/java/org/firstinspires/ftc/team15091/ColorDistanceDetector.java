package org.firstinspires.ftc.team15091;

public class ColorDistanceDetector implements IObjectDetector {
    private ColorDetector colorDetector;
    private DistanceDetector distanceDetector;

    public ColorDistanceDetector(ColorDetector cDetector, DistanceDetector dDetector) {
        colorDetector = cDetector;
        distanceDetector = dDetector;
    }

    public boolean objectDetected() {
        return colorDetector.objectDetected() || distanceDetector.objectDetected();
    }

    public void dispose() {
    }

    public void reset() {
        colorDetector.reset();
        distanceDetector.reset();
    }
}
