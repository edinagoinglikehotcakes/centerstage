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

package hotcakes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */
@TeleOp(name = "CenterstageDc", group = "Competition")
//@Disabled
public class CenterstageDrive extends LinearOpMode {
    private RobotHardware robotHardware;
    private MotorControl motorControl;
    private ElapsedTime runtime = new ElapsedTime();
    private final double MAX_POWER = .9;
    private final double SLOW_POWER = .3;
    private double motorPower = MAX_POWER;
    private double denominator = 0;
    GamepadEx gamePadEx1;
    GamepadEx gamePadEx2;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;


    @Override
    public void runOpMode() {
        gamePadEx1 = new GamepadEx(gamepad1);
        gamePadEx2 = new GamepadEx(gamepad2);
        TriggerReader triggerRight = new TriggerReader(gamePadEx1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader triggerLeft = new TriggerReader(gamePadEx1, GamepadKeys.Trigger.LEFT_TRIGGER);
        robotHardware = new RobotHardware(this);
        robotHardware.init();
        motorControl = new MotorControl(robotHardware);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robotHardware.Frontleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.Frontright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.Backleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.Backright.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.ArmAngle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.ArmAngle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamePadEx1.readButtons();
            gamePadEx2.readButtons();
            // trigger reader is the right trigger and trigger1 is the left one
            triggerRight.readValue();
            triggerLeft.readValue();

            //Joystick movement
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x * 1.1;
            yaw = gamepad1.right_stick_x;
            // Reduce speed with left bumper
            motorPower = gamepad1.left_bumper ? SLOW_POWER : MAX_POWER;
            // Gamepad 1 changing the arm angles
            if (gamepad1.dpad_up) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.BACKDROP);
            }

            if (gamepad1.dpad_down) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.PICKUP);
            }

            if (gamepad1.dpad_right) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.DRIVE);
            }

            if (gamepad1.dpad_left) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.MAXANGLE);
            }

            // Hanging gamepad 1
            if (triggerRight.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.HANGING);
            }

            if (triggerLeft.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.DOWN);
            }

            // Controls gripper flipping gamepad 2
            if (gamepad2.dpad_up) {
                motorControl.flipGripper(MotorControl.GripperAngle.BACKSTAGE);
            }

            if (gamepad2.dpad_down) {
                motorControl.flipGripper(MotorControl.GripperAngle.PICKUP);
            }

            // Gripper control gamepad 2
            if (gamepad2.left_bumper) {
                motorControl.moveGripper(MotorControl.GripperState.CLOSE, MotorControl.GripperSelection.BOTH);
            }

            if (gamepad2.right_bumper) {
                motorControl.moveGripper(MotorControl.GripperState.OPEN, MotorControl.GripperSelection.BOTH);
            }

            // Controls arm extension gamepad 2
            if (gamepad2.y) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.BACKDROP);
            }

            // Gamepad 2 arm extension for retracting
            if (gamepad2.b) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.RETRACT);
            }

            // arm extension for picking up
            if (gamepad2.x) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.PICKUP);
            }

            // Launching plane, only during end game.
            if (gamepad1.x && runtime.seconds() >= 90) {
                robotHardware.DroneLaunch.setPosition(0.35);
            }

            // Waiting launch position, resets the launch servo.
            if (gamepad1.back) {
                motorControl.changeLaunchAngle(MotorControl.LaunchAngle.WAITING);
            }

            // Just trigger launch, not angle adjustment. For testing.
            if (gamepad2.start) {
                motorControl.launchPlane(MotorControl.LaunchState.LAUNCH);
            }

            motorControl.drive(axial, lateral, yaw, motorPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm position", robotHardware.ArmMotor.getCurrentPosition());
            telemetry.addData("Arm angle position", robotHardware.ArmAngle.getCurrentPosition());
            telemetry.addData("Game pad 2 right stick Y", gamepad2.right_stick_y);
            telemetry.addData("Game pad 2 left stick Y", gamepad2.left_stick_y);
            telemetry.addData("Winch motor position", robotHardware.HangMotor.getCurrentPosition());
            telemetry.addData("gripper pos left", robotHardware.GripperLeft.getPosition());
            telemetry.addData("gripper pos right", robotHardware.GripperRight.getPosition());
            telemetry.update();
        }
    }
}