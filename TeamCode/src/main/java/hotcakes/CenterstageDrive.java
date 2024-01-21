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
                gamePadEx, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        TriggerReader triggerLeft = new TriggerReader(
                gamePadEx, GamepadKeys.Trigger.LEFT_TRIGGER
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
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gamePadEx.readButtons();
            gamePadEx2.readButtons();
            triggerRight.readValue();
            triggerLeft.readValue();

            //Joystick movement
            double axial = -gamePadEx.getLeftY();
            double lateral = gamePadEx.getLeftX() * 1.1;
            double yaw = gamePadEx.getRightX();
// Reduce speed
            if (gamePadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                maxPower = 0.3;
            } else {
                maxPower = 0.9;
            }
//           Controls for arm up/down
            if (gamePadEx.wasJustPressed(GamepadKeys.Button.B)) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.DOWN);
            }
            if (gamePadEx.wasJustPressed(GamepadKeys.Button.Y)) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.UP);
            }

            if (gamePadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.BACKDROP);
            }

            if (gamePadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.PICKUP);
            }

            if (gamePadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.DRIVE);
            }
//            Controls Gripper
//            if (gamepad2.left_bumper) {
//                motorControl.moveGripper(MotorControl.gripperCurrentState.CLOSE, MotorControl.GRIPPER_SELECTION.BOTH);
//            }
//            if (gamepad2.right_bumper) {
//                motorControl.moveGripper(MotorControl.gripperCurrentState.OPEN, MotorControl.GRIPPER_SELECTION.BOTH);
//            }
            if (gamePadEx.wasJustPressed(GamepadKeys.Button.A)) {
                motorControl.launchPlane(MotorControl.LaunchState.WAITING);
            }

            if (gamePadEx.wasJustPressed(GamepadKeys.Button.X)) {
                motorControl.launchPlane(MotorControl.LaunchState.LAUNCH);
            }

            if (triggerRight.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.HANGING);
            }

            if (triggerLeft.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.DOWN);
            }

//              This line is the whole drive code from the Motor Control class
            motorControl.drive(axial, lateral, yaw, maxPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position", robotHardware.ArmMotor.getCurrentPosition());
            telemetry.addData("right stick value", gamepad2.right_stick_y);
            telemetry.addData("left stick value", gamepad2.left_stick_y);
            telemetry.addData("Arm servo position", robotHardware.ArmAngle.getPosition());
            telemetry.addData("Winch motor position", robotHardware.HangMotor.getCurrentPosition());
//            telemetry.addData("gripper pos left", robotHardware.GripperLeft.getPosition());
//            telemetry.addData("gripper pos right", robotHardware.GripperRight.getPosition());
            telemetry.update();
        }
    }

    private void launchDrone() {
    }
}
 