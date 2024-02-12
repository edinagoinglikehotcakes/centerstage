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
    private double maxPower = .9;
    private double denominator = 0;
    GamepadEx gamePadEx1;
    GamepadEx gamePadEx2;

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
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.ArmAngle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robotHardware.ArmAngle.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            gamePadEx1.readButtons();
            gamePadEx2.readButtons();
            // trigger reader is the right trigger and trigger1 is the left one
            triggerRight.readValue();
            triggerLeft.readValue();

            //Joystick movement
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x * 1.1;
            double yaw = gamepad1.right_stick_x;
            // Reduce speed
            if (gamePadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                maxPower = 0.3;
            } else {
                maxPower = 0.9;
            }
            // Gamepad 1 changing the arm angles
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.BACKDROP);
            }
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.PICKUP);
            }
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.DRIVE);
            }
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                motorControl.changeArmAngle(MotorControl.ArmAngle.MAXANGLE);
            }
            // Launching plane gamepad 1
            if (gamepad1.a) {
                motorControl.launchPlane(MotorControl.LaunchState.WAITING);
            }
            if (gamepad1.x) {
                robotHardware.DroneLaunch.setPosition(0.35);
            }
            // Hanging gamepad 1
            if (triggerRight.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.HANGING);
            }
            if (triggerLeft.wasJustPressed()) {
                motorControl.hangRobot(MotorControl.HangState.DOWN);
            }
            // Controls gripper flipping gamepad 2
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                motorControl.flipGripper(MotorControl.GripperAngle.BACKSTAGE);
            }
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                motorControl.flipGripper(MotorControl.GripperAngle.PICKUP);
            }
            // Controls Gripper gamepad 2
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                motorControl.moveGripper(MotorControl.GripperState.CLOSE, MotorControl.GripperSelection.BOTH);
            }
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                motorControl.moveGripper(MotorControl.GripperState.OPEN, MotorControl.GripperSelection.BOTH);
            }
            // Controls backdrop arm extension gamepad 2
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.BACKDROP);
            }
            // Gamepad 2 arm extension for retracting
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.RETRACT);
            }
            // arm extension for picking up
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                motorControl.mobilizeArm(MotorControl.ArmExtension.PICKUP);
            }
            // CHANGE LAUNCH ANGLE
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.START)) {
                motorControl.changeLaunchAngle(MotorControl.LaunchAngle.LAUNCH);
            }
            if (gamePadEx1.wasJustPressed(GamepadKeys.Button.BACK)) {
                motorControl.changeLaunchAngle(MotorControl.LaunchAngle.WAITING);
            }
            if (gamePadEx2.wasJustPressed(GamepadKeys.Button.START)) {
                motorControl.launchPlane(MotorControl.LaunchState.LAUNCH);
            }
            // This line is the whole drive code from the Motor Control class
            motorControl.drive(axial, lateral, yaw, maxPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Arm Position", robotHardware.ArmMotor.getCurrentPosition());
            telemetry.addData("right stick value", gamepad2.right_stick_y);
            telemetry.addData("left stick value", gamepad2.left_stick_y);
            telemetry.addData("Arm servo position", robotHardware.ArmAngle.getCurrentPosition());
            telemetry.addData("Winch motor position", robotHardware.HangMotor.getCurrentPosition());
            telemetry.addData("gripper pos left", robotHardware.GripperLeft.getPosition());
            telemetry.addData("gripper pos right", robotHardware.GripperRight.getPosition());
            telemetry.update();
        }
    }
}
 