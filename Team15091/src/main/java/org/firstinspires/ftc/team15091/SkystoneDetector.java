package org.firstinspires.ftc.team15091;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector implements IObjectDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AYuhzJv/////AAABmeg8aNmI7008toUzc6hhMIgtAFhnIF3mx6avMFksv/Jm2qTivu8ialyM7onEEB2F7JVsgl3MXkaV1ZooYDc3fvW6+c3motqWyDpAWC9Z8PJxUHcn+6o+iMkwSyPNYQYQLohALfEcQ0I/HmCxRj7SraSmcssIzSbvdnvMHspynndi47Tg+Ie66vc2EPVhSf9oMOGfIY9KqzhfIfY2QhG58tFYPusqypoqWW6gKBe4NkdcVMIh0HvirCGID/zdeJ9xyp8EoCN3JwR8v9IwGzZHdz0vDVOcnmcJaiYuiAXtpuQklPJGclN93y/W9UBVciFQsxmdcxD4jjsx9XGHANHHCUiyeFs1Y6SI8/0JcDeeMCK9\";";
    private VuforiaLocalizer vuforia = null;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private LinearOpMode opMode;
    List<Recognition> skytoneRecognitions;
    private boolean skytoneDetected = false;
    float visibleMidpoint = -1f;
    float thresholdMin = 180L, thresholdMax = 550L;

    public SkystoneDetector(LinearOpMode opMode, float thresholdMin, float thresholdMax) {
        this.opMode = opMode;
        this.thresholdMin = thresholdMin;
        this.thresholdMax = thresholdMax;
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    public boolean objectDetected() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                skytoneRecognitions = new ArrayList<>();
                skytoneDetected = false;
                for (Recognition recognition : updatedRecognitions) {
                    float recognitionLeft = Math.max(recognition.getLeft(), 0L);
                    float recognitionWidth = recognition.getRight() - recognitionLeft;
                    float recognitionMid = recognitionLeft + (recognitionWidth / 2L);

                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT && recognitionMid > thresholdMin && recognitionMid < thresholdMax) {
                        visibleMidpoint = recognitionMid;
                        skytoneDetected = true;
                        skytoneRecognitions.add(recognition);
                    }
                }
            }
        }

        return skytoneDetected;
    }

    public void reset() {
        skytoneDetected = false;
        visibleMidpoint = -1f;
    }

    public void dispose() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7d;
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
