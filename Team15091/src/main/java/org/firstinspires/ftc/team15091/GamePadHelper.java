package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class GamePadHelper {
    private ElapsedTime driveTime;
    private Gamepad gamepad;
    private AztecRobot robot;
    private boolean xPressed = false, yPressed = false, aPressed = false, bPressed = false;

    public GamePadHelper(Gamepad gamepad, AztecRobot aztecRobot) {
        this.gamepad = gamepad;
        driveTime = new ElapsedTime();
        robot = aztecRobot;
    }

    void processDpad() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerFL = 0d;
        double powerFR = 0d;
        double powerRR = 0d;
        double powerRL = 0d;
        double drive = 0.2d + (driveTime.milliseconds() / 1200d);
        double powerToSet = Range.clip(drive, -1d, 1d);

        if (gamepad.dpad_down && gamepad.left_bumper) {
            powerFL = powerRR = -powerToSet;
        } else if (gamepad.dpad_down && gamepad.right_bumper) {
            powerFR = powerRL = -powerToSet;
        } else if (gamepad.dpad_up && gamepad.left_bumper) {
            powerFR = powerRL = powerToSet;
        } else if (gamepad.dpad_up && gamepad.right_bumper) {
            powerFL = powerRR = powerToSet;
        } else if (gamepad.dpad_up) {
            powerFL = powerFR = powerRL = powerRR = powerToSet;
        } else if (gamepad.dpad_down) {
            powerFL = powerFR = powerRL = powerRR = -powerToSet;
        } else if (gamepad.dpad_right) {
            powerFL = powerRL = powerToSet;
            powerFR = powerRR = -powerToSet;
        } else if (gamepad.dpad_left) {
            powerFL = powerRL = -powerToSet;
            powerFR = powerRR = powerToSet;
        } else if (gamepad.left_bumper) {
            powerFL = powerRR = -powerToSet;
            powerFR = powerRL = powerToSet;
        } else if (gamepad.right_bumper) {
            powerFL = powerRR = powerToSet;
            powerFR = powerRL = -powerToSet;
        } else {
            driveTime.reset();
        }

        robot.setDrivePower(powerFL, powerFR, powerRL, powerRR);
    }

    void processLeftTrigger() {
        double powerArm = 0;
        if (gamepad.left_trigger > 0d) { //Arm going up
            powerArm = Range.clip(gamepad.left_trigger, 0d, 1d);
        } else if (gamepad.right_trigger > 0d) { //Arm going down
            powerArm = -Range.clip(gamepad.right_trigger, 0d, 1d);
        }

        robot.setArmPower(powerArm);
    }

    void processXYAB() {
        if (gamepad.x) {
            if (xPressed == false) {
                xPressed = true;
                robot.modifyHook();
            }
        } else {
            xPressed = false;
        }

        if (gamepad.b) {
            if (bPressed != true) {
                bPressed = true;
                robot.openClaw();
            }
        } else {
            bPressed = false;
        }

        if (gamepad.y) {
            if (yPressed != true) {
                yPressed = true;
                robot.turnClaw();
            }
        } else {
            yPressed = false;
        }
    }
}
