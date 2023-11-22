package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorControl {
    //    set limits
//    private final int ARM_TURN_LIMIT = 200;
    private final int ARM_LIMIT = -2180;
//    private final double TURN_SPEED = 0.3;
    private final double ARM_SPEED = 0.5;
    private final double GRIPPER_LIMIT = 0.6;
    private final double GRIPPER_CLOSE_VALUE = 0.2;
    //    TODO CHANGE SOME OF THESE VALUES ACCORDING TO TUNING
    private final double ARM_SERVO_PICKUP_POSITION = 0.9;
    private final double ARM_SERVO_DROP_POSITION = 0.1;
    private final double ARM_SERVO_LIMIT = 0.8;
    private final double SERVO_FLIPPER_DROP_POSITION = 0;
    private final double SERVO_FLIPPER_PICKUP_POSITION = 0.36;
    private RobotHardware robotHardware;

    //    Which direction the arm is currently going
    public enum armDirection {
        LEFT,
        RIGHT,
        STOP,
    }

    public enum armMovingDirection {
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

//    public void rotateArm(armDirection direction) {
////        Set direction of arm and call Enum and sets limits to the arm rotation
//        if (direction == armDirection.LEFT) {
//            if (robotHardware.TurnMotor.getCurrentPosition() >= -ARM_TURN_LIMIT) {
//                robotHardware.TurnMotor.setPower(-TURN_SPEED);
//            } else {
//                robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//                robotHardware.TurnMotor.setPower(0);
//            }
//        }
//
//        if (direction == armDirection.RIGHT) {
//            if (robotHardware.TurnMotor.getCurrentPosition() <= ARM_TURN_LIMIT) {
//                robotHardware.TurnMotor.setPower(TURN_SPEED);
//            } else {
//                robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//                robotHardware.TurnMotor.setPower(0);
//            }
//        }
//        if (direction == armDirection.STOP) {
//            robotHardware.TurnMotor.setPower(0);
//            robotHardware.TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        }
//    }

    //    ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(armMovingDirection movingDirection) {
        if (movingDirection == armMovingDirection.UP) {
            if (robotHardware.ArmMotor.getCurrentPosition() > ARM_LIMIT) {
                robotHardware.ArmMotor.setPower(-ARM_SPEED);
            }
        }
        if (movingDirection == armMovingDirection.DOWN) {
            if (robotHardware.ArmMotor.getCurrentPosition() < -70) {
                robotHardware.ArmMotor.setPower(ARM_SPEED);
            }
        }
        if (movingDirection == armMovingDirection.NONE) {
            robotHardware.ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robotHardware.ArmMotor.setPower(0);
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

    public void drive(double axial, double lateral, double yaw,  double denominator, double maxPower) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        denominator = Math.max(Math.abs(lateral) + Math.abs(axial) + Math.abs(yaw), 1);
        robotHardware.Frontleft.setPower(((axial + lateral + yaw) / denominator) * maxPower);
        robotHardware.Backleft.setPower((((axial - lateral) + yaw) / denominator) * maxPower);
        robotHardware.Frontright.setPower((((axial - lateral) - yaw) / denominator) * maxPower);
        robotHardware.Backright.setPower((((axial + lateral) - yaw) / denominator) * maxPower);
    }
}

