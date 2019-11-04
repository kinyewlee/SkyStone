package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceDetector implements IObjectDetector {
    private DistanceSensor sensorRange;
    private double threshold = 2;
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Set distance detector
     * @param sensorToUse
     * @param threshold in CM
     */
    public DistanceDetector(DistanceSensor sensorToUse, double threshold) {
        sensorRange = sensorToUse;
        this.threshold = threshold;
    }

    public boolean objectDetected() {
        double currentDistance = sensorRange.getDistance(DistanceUnit.CM);
        boolean detected = false;

        if (currentDistance < threshold) {
            //if the detection last for 90 millisesoncds then sounds like real
            if (runtime.milliseconds() > 90d) {
                detected = true;
            }
        }
        else {
            runtime.reset();
        }
        return detected;
    }

    public void dispose() {

    }
}
