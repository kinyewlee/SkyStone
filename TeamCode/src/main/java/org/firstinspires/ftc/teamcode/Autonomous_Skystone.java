package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "Skystone: Test", group = "Skystone")
public class Autonomous_Skystone extends Autonomous_Base {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new AztecRobot();
        robot.init(hardwareMap);
        robot.resetDrive();

        while (!robot.imu.isGyroCalibrated()) {
            idle();
        }

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        if (opModeIsActive()) {
            SkystoneDetector skystoneDetector = new SkystoneDetector(tfod);
            gyroSlide(0.01d, 1000d, skystoneDetector);
            //gyroDrive(1d, -12d, 0d);
            //gyroSlide(1d, 12d);

            VuforiaSkyStone t = new VuforiaSkyStone();


        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
