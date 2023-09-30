/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name="CenterstageDc")
//@Disabled
public class CenterstageDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Motor and servo identification
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Frontright;
    private DcMotor Backright;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        Frontleft  = hardwareMap.get(DcMotor.class, "Frontleft");
        Backleft  = hardwareMap.get(DcMotor.class, "Backleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backright = hardwareMap.get(DcMotor.class, "Backright");

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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //define variables
            double maxPower = 0.9;
            double denominator;
            //Joystick movement
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x*1.1;

            if (gamepad1.left_bumper) {
                maxPower = 0.3;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
//            If the code below does not work, comment it out. Uncomment out the code below this code below.
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(axial), Math.abs(lateral), Math.abs(yaw))), 1));
            Frontleft.setPower(((axial + lateral + yaw) /denominator)* maxPower);
            Backleft.setPower((((axial - lateral) + yaw) /denominator)* maxPower);
            Frontright.setPower((((axial - lateral) - yaw) /denominator) * maxPower);
            Backright.setPower((((axial + lateral) - yaw) /denominator) * maxPower);


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
//            maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
//            maxPower = Math.max(maxPower, Math.abs(rightBackPower));
//
//            if (maxPower > 1.0) {
//                leftFrontPower  /= maxPower;
//                rightFrontPower /= maxPower;
//                leftBackPower   /= maxPower;
//                rightBackPower  /= maxPower;
//            }
//
//            Frontleft.setPower(leftFrontPower);
//            Frontright.setPower(rightFrontPower);
//            Backleft.setPower(leftBackPower);
//            Backright.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
