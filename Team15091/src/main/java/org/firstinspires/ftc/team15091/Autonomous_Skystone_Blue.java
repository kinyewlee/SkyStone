package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Skystone: Blue", group = "Skystone")
public class Autonomous_Skystone_Blue extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot(hardwareMap);

        RobotDriver robotDriver = new RobotDriver(robot, this);
        SkystoneDetector skystoneDetector = new SkystoneDetector(this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 30d, 100d);

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
            robotDriver.gyroSlide(0.48d, -60d, 0d, 3d, skystoneDetector);
            robotDriver.gyroDrive(0.3d, 6d, 0d, 2d, distanceDetector);

            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 3d);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(300L);
            robotDriver.gyroDrive(0.7d, -12d, 0d, 2d, null);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(300L);

            robotDriver.moveArm(2.85d, 4d);
        }
    }
}
