package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Java Autonomous Camera", group = "Auto")
public class JavaAutonomousCamera extends LinearOpMode {

SpikeDetection spikeDetection;
OpenCvCamera camera;
String webcamName = "webcam1";

    private SpikeDetection.Position parkLocation;
    private DcMotor BackleftAsDcMotor;
    private DcMotor Backright;
    private DcMotor FrontleftAsDcMotor;
    private DcMotor Frontright;

    int blpos;
    int brpos;
    int flpos;
    int frpos;
    int Step_;


    /**
     * Describe this function...
     */
    private void drive(double blTarget, double brTarget, double flTarget, double frTarget, double speed) {
        blpos += blTarget;
        brpos += brTarget;
        flpos += flTarget;
        frpos += frTarget;
        BackleftAsDcMotor.setTargetPosition(blpos);
        Backright.setTargetPosition(brpos);
        FrontleftAsDcMotor.setTargetPosition(flpos);
        Frontright.setTargetPosition(frpos);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setPower(speed);
        Backright.setPower(speed);
        FrontleftAsDcMotor.setPower(speed);
        Frontright.setPower(speed);
        while (opModeIsActive() && BackleftAsDcMotor.isBusy() && Backright.isBusy() && FrontleftAsDcMotor.isBusy() && Frontright.isBusy()) {
            idle();
        }
    }

    @Override
    public void runOpMode() {

    }
}