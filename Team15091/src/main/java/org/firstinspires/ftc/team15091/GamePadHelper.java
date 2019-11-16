package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class GamePadHelper {
    private ElapsedTime driveTime;
    private Gamepad gamepad;
    private AztecRobot robot;
    private boolean xPressed = false, yPressed = false, aPressed = false, bPressed = false;
    private GamePadOrientation gamePadOrientation;

    public GamePadHelper(Gamepad gamepad, AztecRobot aztecRobot, GamePadOrientation gamePadOrientation) {
        this.gamepad = gamepad;
        driveTime = new ElapsedTime();
        robot = aztecRobot;
        this.gamePadOrientation = gamePadOrientation;
    }

    void processJoystick() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double powerFL, powerFR, powerRR, powerRL;

        // POV Mode uses left stick y to go forward, and left stick x to turn.
        // right stick x to move side way
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = Range.scale(-gamepad.left_stick_y - gamepad.right_stick_y, -1d, 1d, -0.9d, 0.9d);
        double turn = gamepad.left_stick_x;
        double side = gamepad.right_stick_x;


        if (gamePadOrientation == GamePadOrientation.REVERSE) {
            drive *= -1d;
            side *= -1d;
        }

        powerFL = Range.clip(drive + turn + side, -1.0, 1.0);
        powerRL = Range.clip(drive + turn - side, -1.0, 1.0);
        powerFR = Range.clip(drive - turn - side, -1.0, 1.0);
        powerRR = Range.clip(drive - turn + side, -1.0, 1.0);

        if (gamepad.left_bumper) {
            powerFL *= 0.75d;
            powerRL *= 0.75d;
            powerFR *= 0.75d;
            powerRR *= 0.75d;
        }

        // Send calculated power to wheels
        robot.setDrivePower(powerFL, powerFR, powerRL, powerRR);
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

    void processXYAB() {
        if (gamepad.x) {
            if (xPressed == false) {
                xPressed = true;
                robot.modifyHook();
            }
        } else {
            xPressed = false;
        }

        processB();

        if (gamepad.y) {
            if (yPressed != true) {
                yPressed = true;
                robot.turnClaw();
            }
        } else {
            yPressed = false;
        }
    }

    void processB() {
        if (gamepad.b) {
            if (bPressed != true) {
                bPressed = true;
                robot.openClaw();
            }
        } else {
            bPressed = false;
        }
    }
}
