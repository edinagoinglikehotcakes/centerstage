package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotHardware {
    private OpMode myOpMode = null;
    public DcMotorEx Frontleft = null;
    public DcMotorEx Backleft = null;
    public DcMotorEx Frontright = null;
    public DcMotorEx Backright = null;

    public RobotHardware (OpMode opMode) {myOpMode = opMode;}

    public void init() {
        Frontleft  = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontleft");
        Backleft  = myOpMode.hardwareMap.get(DcMotorEx.class, "Backleft");
        Frontright = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontright");
        Backright = myOpMode.hardwareMap.get(DcMotorEx.class, "Backright");

        //Motor direction

        Frontleft.setDirection(DcMotor.Direction.REVERSE);
        Backleft.setDirection(DcMotor.Direction.REVERSE);
        Frontright.setDirection(DcMotor.Direction.FORWARD);
        Backright.setDirection(DcMotor.Direction.FORWARD);

        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
