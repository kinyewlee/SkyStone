package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SkystoneDetector implements IRobotDriver {
    private TFObjectDetector tfod;

    public SkystoneDetector(TFObjectDetector tfObjectDetector) {
        tfod = tfObjectDetector;
    }

    @Override
    public boolean applyBrake() {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                return updatedRecognitions.size() > 0;
            }
        }

        return false;
    }

}
