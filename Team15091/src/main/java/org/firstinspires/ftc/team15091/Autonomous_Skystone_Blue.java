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
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 2d);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(600L);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(500L);

            robotDriver.moveArm(2.85d, 2d);
        }
    }
}
