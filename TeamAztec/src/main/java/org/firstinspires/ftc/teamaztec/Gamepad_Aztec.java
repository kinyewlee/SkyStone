package org.firstinspires.ftc.teamaztec;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad", group = "Linear Opmode")
public class Gamepad_Aztec extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Hardware_Aztec robot = new Hardware_Aztec(hardwareMap);
        Gamepad_Helper gamepadHelper1 = new Gamepad_Helper(gamepad1, robot);
        Gamepad_Helper gamepadHelper2 = new Gamepad_Helper(gamepad2, robot);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    telemetry.addData("Heading: ", "%.4f", robot.getHeading());
                    telemetry.update();
                }
            }
        }.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamepadHelper1.processLeftJoystick();
            gamepadHelper1.processLeftTrigger();
            gamepadHelper1.processXYAB();

            gamepadHelper2.processLeftJoystick();
            gamepadHelper2.processLeftTrigger();
            gamepadHelper2.processXYAB();
        }
    }
}
