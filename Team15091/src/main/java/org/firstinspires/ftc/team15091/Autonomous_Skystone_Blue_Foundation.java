package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Autonomous(name = "Skystone: Blue Foundation", group = "Skystone")
public class Autonomous_Skystone_Blue_Foundation extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        RobotDriver robotDriver = new RobotDriver(robot, this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 16d, 90d);
        ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        telemetry.addData(">", "Press Play to start op mode");
        Telemetry.Item headingItem = telemetry.addData("Heading: ", "%.4f", robot.getHeading());
        Telemetry.Item detectorItem = telemetry.addData("Detected: ", distanceDetector.objectDetected());
        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            headingItem.setValue("%.4f", robot.getHeading());
            detectorItem.setValue(distanceDetector.objectDetected());
            telemetry.update();
            idle();
        }

        if (opModeIsActive()) {
            robotDriver.setHook(HookPosition.MIDDLE);
            robotDriver.setArmAngle(2.95d, 1d);

            //move to foundation
            robotDriver.gyroDrive(0.9d, -24d, 0d, 2d, null);
            robotDriver.gyroSlide(0.9d, -22.5d, 0d, 2d, null);
            robotDriver.gyroDrive(0.2d, -18.5d, 0d, 3d, null);

            //attach to foundation
            robotDriver.setHook(HookPosition.DOWN);
            sleep(2000L);

            //move foundation to building zone
            robotDriver.gyroDrive(0.9d, 18d, 0d, 2d, null);
            robotDriver.gyroSlide(0.9d, 24d, 0d, 2d, null);
            robotDriver.gyroTurn(0.9d, 90d, 2d);
            robotDriver.gyroDrive(0.9d, -16d, 90d, 2d, null);

            //detach foundation
            robotDriver.setHook(HookPosition.MIDDLE);
            sleep(200L);

            //move under skybridge
            robotDriver.gyroSlide(0.9d, -32d, 90d, 3d, null);

            //if another robot blocking under skybridge stop
            boolean stepFinish = robotDriver.gyroDrive(0.4d, 34d, 90d, 4d, distanceDetector);
            if (!stepFinish) {
                robotDriver.gyroSlide(0.4d, 30d, 90d, 3d, null);
            }
            robotDriver.gyroDrive(0.4d, 20d, 90d, 2d, colorDetector);
        }
    }
}
