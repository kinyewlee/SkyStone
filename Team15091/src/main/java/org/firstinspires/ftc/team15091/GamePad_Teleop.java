/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "GamePad", group = "Linear Opmode")
//Disabled
public class GamePad_Teleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap, false);
        GamePadHelper gamePadHelper1 = new GamePadHelper(gamepad1, robot);
        GamePadHelper gamePadHelper2 = new GamePadHelper(gamepad2, robot);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("Arm", "%.4f", robot.getArmAngle());
                telemetry.update();
            }
        }
        ).start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamePadHelper1.processJoystick();
            gamePadHelper1.processXYAB();

            gamePadHelper2.processJoystick();
            //gamePadHelper2.processXYAB();

            double powerArm = 0;
            if (gamepad1.left_trigger > 0d) { //Arm going up
                powerArm = Range.clip(Math.pow(gamepad1.left_trigger, 2), 0d, 1d);
            } else if (gamepad1.right_trigger > 0d) { //Arm going down
                powerArm = -Range.clip(Math.pow(gamepad1.right_trigger, 2), 0d, 1d);
            } else if (gamepad2.left_trigger > 0d) { //Arm going up
                powerArm = Range.clip(gamepad2.left_trigger, 0d, 1d);
            } else if (gamepad2.right_trigger > 0d) { //Arm going down
                powerArm = -Range.clip(gamepad2.right_trigger, 0d, 1d);
            }

            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                powerArm *= 0.4d;
            }

            robot.setArmPower(powerArm);

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                robot.winchUp();
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                robot.winchDown();
            } else {
                robot.winchStop();
            }
        }
    }
}
