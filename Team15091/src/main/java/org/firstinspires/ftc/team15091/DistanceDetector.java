package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceDetector implements IObjectDetector {
    private DistanceSensor sensorRange;
    private double threshold = 2;

    public DistanceDetector(DistanceSensor sensorToUse, double threshold) {
        sensorRange = sensorToUse;
        this.threshold = threshold;
    }

    public boolean objectDetected() {


        double currentDistance = sensorRange.getDistance(DistanceUnit.CM);
        if (currentDistance < threshold) {
            return true;
        }
        return false;
    }

    public void dispose() {

    }
}
