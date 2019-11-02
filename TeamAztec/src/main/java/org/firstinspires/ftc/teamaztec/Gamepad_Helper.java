package org.firstinspires.ftc.teamaztec;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Gamepad_Helper {
    Gamepad gamepad;
    Hardware_Aztec robot;
    private boolean xPressed = false, yPressed = false, aPressed = false, bPressed = false;

    Gamepad_Helper(Gamepad gamepad, Hardware_Aztec robot) {
        this.gamepad = gamepad;
        this.robot = robot;
    }

    void processLeftJoystick() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = Range.clip(-gamepad.left_stick_y - gamepad.right_stick_y, -1d, 1d);
        double turn = gamepad.left_stick_x;

        // Combine drive and turn for blended motion.
        leftPower = drive + turn;
        rightPower = drive - turn;

        // Send calculated power to wheels
        robot.setPower(leftPower, rightPower);
    }

    void processLeftTrigger() {
        double powerArm = 0;
        if (gamepad.left_trigger > 0d) { //Arm going up
            powerArm = Range.clip(gamepad.left_trigger, 0d, 1d);
        } else if (gamepad.right_trigger > 0d) { //Arm going down
            powerArm = -Range.clip(gamepad.right_trigger, 0d, 1d);
        }

        if (robot.getArmAngle() > 1.05d) {
            double wristPosition = Range.scale(robot.getArmAngle(), 1.06d, robot.ARM_MAX, 0d, 0.55d);
            robot.servoWrist.setPosition(wristPosition);
        } else if (robot.getArmAngle() < 0.95d) {
            robot.servoWrist.setPosition(1d);
        }

        robot.setArmPower(powerArm);
    }

    void processXYAB() {
        if (gamepad.y) {
            if (yPressed != true) {
                yPressed = true;
                robot.turnClaw();
            }
        } else {
            yPressed = false;
        }

        if (gamepad.b) {
            if (bPressed != true) {
                bPressed = true;
                robot.toggleClaw();
            }
        } else {
            bPressed = false;
        }
    }
}
