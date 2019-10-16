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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "GamePad", group = "Linear Opmode")
//@Disabled
public class GamePad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double maxPower = 1d;
    ElapsedTime driveTime;
    boolean previousX = false;


    public GamePad() {
        driveTime = new ElapsedTime();
    }

    @Override
    public void runOpMode() {

        final AztecRobot myRobot = new AztecRobot();
        myRobot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    telemetry.addData("Arm:", "pos (%.3f)", myRobot.getArmAngle());
                    telemetry.addData("Heading: ", "%.4f", myRobot.getHeading());
                    telemetry.update();
                }
            }
        }.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double powerFL = 0d;
            double powerFR = 0d;
            double powerRR = 0d;
            double powerRL = 0d;
            double drive = 0.2d + (driveTime.milliseconds() / 1200d);
            double powerToSet = Range.clip(drive, -1d, 1d);

            if( gamepad1.dpad_down && gamepad1.left_bumper) {
                powerFL = powerRR = -powerToSet;
            } else if( gamepad1.dpad_down && gamepad1.right_bumper) {
                powerFR = powerRL = -powerToSet;
            } else if( gamepad1.dpad_up && gamepad1.left_bumper) {
                powerFR = powerRL = powerToSet;
            } else if( gamepad1.dpad_up && gamepad1.right_bumper) {
                powerFL = powerRR = powerToSet;
            } else if (gamepad1.dpad_up) {
                powerFL = powerFR = powerRL = powerRR = powerToSet;
            } else if (gamepad1.dpad_down) {
                powerFL = powerFR = powerRL = powerRR = -powerToSet;
            } else if (gamepad1.dpad_right) {
                powerFL = powerRL = powerToSet;
                powerFR = powerRR = -powerToSet;
            } else if (gamepad1.dpad_left) {
                powerFL = powerRL = -powerToSet;
                powerFR = powerRR = powerToSet;
            } else if (gamepad1.left_bumper) {
                powerFL = powerRR = -powerToSet;
                powerFR = powerRL = powerToSet;
            } else if (gamepad1.right_bumper) {
                powerFL = powerRR = powerToSet;
                powerFR = powerRL = -powerToSet;
            } else {
                driveTime.reset();
            }

            myRobot.setDrivePower(powerFL, powerFR, powerRL, powerRR);

            double powerArm = 0;
            if (gamepad1.left_trigger > 0d) { //Arm going up
                powerArm = Range.clip(gamepad1.left_trigger, 0d, 1d);
            } else if (gamepad1.right_trigger > 0d) { //Arm going down
                powerArm = -Range.clip(gamepad1.right_trigger, 0d, 1d);
            }

            myRobot.setArmPower(powerArm);
            if(gamepad1.x){
                if(previousX!=gamepad1.x) {
                    previousX=gamepad1.x;
                    myRobot.modifyHook();
                }
            }else{
                previousX=gamepad1.x;
            }
        }
    }
}

