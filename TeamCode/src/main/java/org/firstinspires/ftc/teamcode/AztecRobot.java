package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

class AztecRobot {
    private HardwareMap hwMap = null;
    private DcMotor motorFL = null;
    private DcMotor motorRL = null;
    private DcMotor motorFR = null;
    private DcMotor motorRR = null;

    AztecRobot() {
    }


    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        motorFL = hwMap.get(DcMotor.class, "motor_0");
        motorRL = hwMap.get(DcMotor.class, "motor_1");
        motorFR = hwMap.get(DcMotor.class, "motor_3");
        motorRR = hwMap.get(DcMotor.class, "motor_2");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.FORWARD);

        setPower(0,0,0,0);
    }

    void setPower (double powerFL, double powerFR, double powerRL, double powerRR){
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorRL.setPower(powerRL);
        motorRR.setPower(powerRR);
    }
}
