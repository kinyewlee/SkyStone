package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Autonomous(name = "Skystone: Test", group = "Skystone")
public class Autonomous_Skystone extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot();
        robot.init(hardwareMap);
        robot.resetDrive();

        RobotDriver robotDriver = new RobotDriver(robot, this);
        SkystoneDetector skystoneDetector = new SkystoneDetector(this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 20);
        telemetry.addData(">", "Press Play to start op mode");
        Telemetry.Item headingItem = telemetry.addData("Heading: ", "%.4f", robot.getHeading());

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            headingItem.setValue("%.4f", robot.getHeading());
            telemetry.update();
            idle();
        }

        if (opModeIsActive()) {
            //robotDriver.gyroSlide(0.8d, 100d, 0d, skystoneDetector);
            robotDriver.gyroDrive(1d, 100d, 0d, distanceDetector);
            //gyroSlide(1d, 12d);
        }

        skystoneDetector.dispose();
    }
}
