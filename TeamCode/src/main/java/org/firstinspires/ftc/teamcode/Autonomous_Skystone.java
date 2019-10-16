package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Skystone: Test", group = "Skystone")
public class Autonomous_Skystone extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot();
        robot.init(hardwareMap);
        robot.resetDrive();

        while (!robot.imu.isGyroCalibrated()) {
            idle();
        }

        RobotDriver robotDriver = new RobotDriver(robot,this);
        SkystoneDetector skystoneDetector = new SkystoneDetector(this);

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            idle();
        }

        if (opModeIsActive()) {
            robotDriver.gyroSlide(0.01d, 1000d, skystoneDetector);
            //gyroDrive(1d, -12d, 0d);
            //gyroSlide(1d, 12d);
        }

        skystoneDetector.dispose();
    }
}
