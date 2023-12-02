package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorControl {
    //    set limits
//    private final int ARM_TURN_LIMIT = 200;
    private final int ARM_LIMIT = -2180;
    //    private final double TURN_SPEED = 0.3;
    private final double ARM_POWER = 0.5;
    private final double GRIPPER_LIMIT = 0.6;
    private final double GRIPPER_CLOSE_VALUE = 0.2;
    //    TODO CHANGE SOME OF THESE VALUES ACCORDING TO TUNING
    private final double ARM_SERVO_PICKUP_POSITION = 0.9;
    private final int ARM_UP_TARGET_POSITION = 1000;
    private final int ARM_DOWN_TARGET_POSITION = 20;
    private final double ARM_SERVO_DROP_POSITION = 0.1;
    private final double ARM_SERVO_LIMIT = 0.8;
    private final double SERVO_FLIPPER_DROP_POSITION = 0;
    private final double SERVO_FLIPPER_PICKUP_POSITION = 0.36;
    private final double LAUNCHING_SERVO_POSITION = 0.0;
    private final double WAITING_SERVO_POSITION = 0.6;
    private RobotHardware robotHardware;

    //    Which direction the arm is currently going
    public enum LAUNCHSTATE {
        WAITING,
        LAUNCH,
        NONE,
    }

    public enum ARMSTATE {
        UP,
        DOWN,
        NONE,

    }

    public enum gripperCurrentState {
        OPEN,
        CLOSE,
    }

    public enum GRIPPER_SELECTION {
        LEFT,
        RIGHT,
        BOTH,
    }

    public enum armServoState {
        UP,
        DOWN,
        DROP,
        PICKUP,
    }

    public enum servoFlippingState {
        PICKUP,
        DROP,
    }

    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }



    //    ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(ARMSTATE armState) {
        if (armState == ARMSTATE.UP) {
            robotHardware.ArmMotor.setTargetPosition(ARM_UP_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_POWER);

        }
        if (armState == ARMSTATE.DOWN) {
            robotHardware.ArmMotor.setTargetPosition(ARM_DOWN_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(-ARM_POWER);
        }
        if (armState == ARMSTATE.NONE) {
            robotHardware.ArmMotor.setPower(0);
            robotHardware.ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // TODO CODE GRIPPER MOVEMENTS
    public void moveGripper(gripperCurrentState gripperState, GRIPPER_SELECTION gripperSelection) {
        robotHardware.GripperLeft.setDirection(Servo.Direction.REVERSE);
        if (gripperSelection == GRIPPER_SELECTION.BOTH) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperLeft.setPosition(0.6);
                robotHardware.GripperRight.setPosition(0.05);
            }
            if (gripperState == gripperCurrentState.CLOSE) {
                robotHardware.GripperLeft.setPosition(0.2);
                robotHardware.GripperRight.setPosition(0.3);
            }
            return;
        }
        if (gripperSelection == GRIPPER_SELECTION.LEFT) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperLeft.setPosition(0.5);
            } else {
                robotHardware.GripperLeft.setPosition(0.25);
            }
            return;
        }

        if (gripperSelection == GRIPPER_SELECTION.RIGHT) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperRight.setPosition(0.15);
            } else {
                robotHardware.GripperRight.setPosition(0.35);
            }
            return;
        }
    }
    public void launchPlane(LAUNCHSTATE launchstate) {
        if (launchstate == LAUNCHSTATE.LAUNCH) {
            robotHardware.launchServo.setPosition(LAUNCHING_SERVO_POSITION);
        }
        if (launchstate == LAUNCHSTATE.WAITING) {
            robotHardware.launchServo.setPosition(WAITING_SERVO_POSITION);
        }
    }

    public void extendArmServo(armServoState armServoState) {
        if (armServoState == MotorControl.armServoState.UP) {
            robotHardware.ArmServo.setPosition(robotHardware.ArmServo.getPosition() - 0.05);
        }
        if (armServoState == MotorControl.armServoState.DOWN) {
            if (robotHardware.ArmServo.getPosition() > ARM_SERVO_LIMIT) {
                robotHardware.ArmServo.setPosition(robotHardware.ArmServo.getPosition() + 0.05);
            }
        }

        if (armServoState == MotorControl.armServoState.PICKUP) {
            robotHardware.ArmServo.setPosition(ARM_SERVO_PICKUP_POSITION);
        }
        if (armServoState == MotorControl.armServoState.DROP) {
            robotHardware.ArmServo.setPosition(ARM_SERVO_DROP_POSITION);
        }
    }

    public void flipGripper(servoFlippingState flippingState) {
        if (flippingState == servoFlippingState.PICKUP) {
            robotHardware.GripperFlipper.setPosition(SERVO_FLIPPER_PICKUP_POSITION);
        }
        if (flippingState == servoFlippingState.DROP) {
            robotHardware.GripperFlipper.setPosition(SERVO_FLIPPER_DROP_POSITION);
        }
    }

    public void drive(double axial, double lateral, double yaw, double denominator, double maxPower) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        denominator = Math.max(Math.abs(lateral) + Math.abs(axial) + Math.abs(yaw), 1);
        robotHardware.Frontleft.setPower(((axial + lateral + yaw) / denominator) * maxPower);
        robotHardware.Backleft.setPower((((axial - lateral) + yaw) / denominator) * maxPower);
        robotHardware.Frontright.setPower((((axial - lateral) - yaw) / denominator) * maxPower);
        robotHardware.Backright.setPower((((axial + lateral) - yaw) / denominator) * maxPower);
    }
}

