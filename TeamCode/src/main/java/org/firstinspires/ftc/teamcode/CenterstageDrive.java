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

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */

@TeleOp(name = "CenterstageDc")
//@Disabled
public class CenterstageDrive extends LinearOpMode {
    private final double MAX_POWER = 1;
    private RobotHardware robotHardware;
    private MotorControl motorControl;
    private GamepadEx gamepad;
    // Left bumper to implement slow mode.
    private ToggleButtonReader leftBumper;
    private final double SLOW_FACTOR = .3;
    private ElapsedTime runtime = new ElapsedTime();

    //Motor and servo identification

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(this);
        robotHardware.init();
        motorControl = new MotorControl(robotHardware);
        robotHardware.Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gamepad = new GamepadEx(gamepad1);
        ToggleButtonReader leftBumper = new ToggleButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamepad.readButtons();
            //define variables
            //Joystick movement
            double axial = -gamepad.getLeftY();
            double lateral = gamepad.getLeftX();
            double yaw = gamepad.getRightX() * 1.1;

            // Drive slow
            double runPower = leftBumper.getState() ? MAX_POWER * SLOW_FACTOR : MAX_POWER;
            motorControl.drive(axial, lateral, yaw, runPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front", robotHardware.Frontleft.getPower());
            telemetry.addData("Right Front", robotHardware.Frontright.getPower());
            telemetry.addData("Left Back", robotHardware.Backleft.getPower());
            telemetry.addData("Right Back", robotHardware.Backright.getPower());
            telemetry.update();
        }
    }
}
