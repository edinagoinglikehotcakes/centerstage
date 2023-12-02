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

@TeleOp(name = "CenterstageDc")
//@Disabled
public class CenterstageDrive extends LinearOpMode {
    private RobotHardware robotHardware;
    private MotorControl motorControl;
    private ElapsedTime runtime = new ElapsedTime();
    private double maxPower = .9;
    private double denominator = 0;
    GamepadEx gamePadEx;
    GamepadEx gamePadEx2;

    @Override
    public void runOpMode() {
//      This is the code for all of the Hardware
        gamePadEx = new GamepadEx(gamepad1);
        gamePadEx2 = new GamepadEx(gamepad2);
        TriggerReader triggerRight = new TriggerReader(
                gamePadEx2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        TriggerReader triggerLeft = new TriggerReader(
                gamePadEx2, GamepadKeys.Trigger.LEFT_TRIGGER
        );
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
//        robotHardware.TurnMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamePadEx.readButtons();
            gamePadEx2.readButtons();
//            trigger reader is the right trigger and trigger1 is the left one
            triggerRight.readValue();
            triggerLeft.readValue();

            //Joystick movement
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * 1.1;
// Reduce speed
            if (gamepad1.left_bumper) {
                maxPower = 0.3;
            }
//            Controls for the arm
//            if (gamePadEx2.isDown(GamepadKeys.Button.DPAD_LEFT)) {
//                motorControl.rotateArm(MotorControl.armDirection.LEFT);
//            } else if (gamePadEx2.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
//                motorControl.rotateArm(MotorControl.armDirection.STOP);
//            }
//            if (gamePadEx2.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
//                motorControl.rotateArm(MotorControl.armDirection.RIGHT);
//            } else if (gamePadEx2.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
//                motorControl.rotateArm(MotorControl.armDirection.STOP);
//            }
//           Controls for arm up/down
            if (triggerRight.isDown()) {
                motorControl.mobilizeArm(MotorControl.ARMSTATE.UP);
            } else if (triggerRight.wasJustReleased()) {
                motorControl.mobilizeArm(MotorControl.ARMSTATE.NONE);
                if (triggerLeft.isDown()) {
                    motorControl.mobilizeArm(MotorControl.ARMSTATE.DOWN);
                } else if (triggerLeft.wasJustReleased()) {
                    motorControl.mobilizeArm(MotorControl.ARMSTATE.NONE);
                }

            }
//            Controls Gripper
            if (gamepad2.left_bumper) {
                motorControl.moveGripper(MotorControl.gripperCurrentState.CLOSE, MotorControl.GRIPPER_SELECTION.BOTH);
            }
            if (gamepad2.right_bumper) {
                motorControl.moveGripper(MotorControl.gripperCurrentState.OPEN, MotorControl.GRIPPER_SELECTION.BOTH);
            }
//            CODE FOR ARM SERVO
            if (gamepad2.a) {
                motorControl.extendArmServo(MotorControl.armServoState.PICKUP);
            }
            if (gamepad2.y) {
                motorControl.extendArmServo(MotorControl.armServoState.DROP);
            }
            if (gamepad1.a) {
                motorControl.launchPlane(MotorControl.LAUNCHSTATE.WAITING);
            }
            if (gamepad1.x) {
                motorControl.launchPlane(MotorControl.LAUNCHSTATE.LAUNCH);
            }
            if (gamepad2.left_stick_y < 0) {
                motorControl.extendArmServo(MotorControl.armServoState.UP);
            } else if (gamepad2.left_stick_y > 0) {
                motorControl.extendArmServo(MotorControl.armServoState.DOWN);
            }
//              CODE FOR GRIPPER FLIPPER
            if (gamepad2.right_stick_y < 0) {
                motorControl.flipGripper(MotorControl.servoFlippingState.PICKUP);
            }
            if (gamepad2.right_stick_y > 0) {
                motorControl.flipGripper(MotorControl.servoFlippingState.DROP);
            }
//              This line is the whole drive code from the Motor Control class
            motorControl.drive(axial, lateral, yaw, denominator, maxPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position", robotHardware.ArmMotor.getCurrentPosition());
            telemetry.addData("right stick value", gamepad2.right_stick_y);
            telemetry.addData("left stick value", gamepad2.left_stick_y);
            telemetry.addData("gripper pos left", robotHardware.GripperLeft.getPosition());
            telemetry.addData("gripper pos right", robotHardware.GripperRight.getPosition());
            telemetry.update();

        }
    }
}
 