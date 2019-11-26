package org.firstinspires.ftc.teamaztec;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Gamepad_Helper {
    Gamepad gamepad;
    Hardware_Aztec robot;
    ElapsedTime runtime;

    private boolean xPressed = false, yPressed = false, aPressed = false, bPressed = false;

    Gamepad_Helper(Gamepad gamepad, Hardware_Aztec robot) {
        this.gamepad = gamepad;
        this.robot = robot;
        runtime = new ElapsedTime();
    }

    void processLeftJoystick() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerFL, powerFR, powerRR, powerRL;

        // POV Mode uses left stick y to go forward, and left stick x to turn.
        // right stick x to move side way
        // - This uses basic math to combine motions and is easier to drive straight.
        double stickY = -gamepad.left_stick_y;
        if (!gamepad.right_bumper) {
            stickY -= gamepad.right_stick_y;
        }

        double drive = Range.scale(stickY, -1d, 1d, -0.9d, 0.9d);
        double turn = gamepad.left_stick_x;
        double side = gamepad.right_stick_x;


        powerFL = Range.clip(drive + turn + side, -1.0, 1.0);
        powerRL = Range.clip(drive + turn - side, -1.0, 1.0);
        powerFR = Range.clip(drive - turn + side, -1.0, 1.0);
        powerRR = Range.clip(drive - turn - side, -1.0, 1.0);

        if (gamepad.left_bumper) {
            powerFL *= 0.6d;
            powerRL *= 0.6d;
            powerFR *= 0.6d;
            powerRR *= 0.6d;
        }

        // Send calculated power to wheels
        robot.setDrivePower(powerFL, powerFR, powerRL, powerRR);
    }

    void processLeftTrigger() {
        double currentPosition = robot.servoArm.getPosition();
        if (gamepad.left_trigger > 0d && currentPosition > 0d) {
            double interval = Range.scale(gamepad.left_trigger, 0d, 1d, 150d, 15d);
            if (runtime.milliseconds() > interval) {
                double newPosition = currentPosition - 0.01d;
                robot.servoArm.setPosition(newPosition);
                runtime.reset();
            }
        } else if (gamepad.right_trigger > 0d && currentPosition < 1d) {
            double interval = Range.scale(gamepad.right_trigger, 0d, 1d, 150d, 15d);
            if (runtime.milliseconds() > interval) {
                double newPosition = currentPosition + 0.01d;
                robot.servoArm.setPosition(newPosition);
                runtime.reset();
            }
        } else {
            runtime.reset();
        }
    }

    void processXYAB() {

    }
}
