package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorControl {
    //    set limits
    private final int ARM_LIMIT = -2180;
    private final double ARM_POWER = 0.5;
    private final double GRIPPER_OPEN_VALUE = 0.6;
    private final double GRIPPER_CLOSE_VALUE = 0.2;
    //    TODO CHANGE SOME OF THESE VALUES ACCORDING TO TUNING
    private final int ARM_UP_TARGET_POSITION = 700;
    private final int ARM_DOWN_TARGET_POSITION = 20;
    private final double SERVO_FLIPPER_DROP_POSITION = 0;
    private final double SERVO_FLIPPER_PICKUP_POSITION = 0.36;
    //    LAUNCH SERVO
    private final double LAUNCHING_SERVO_POSITION = 0.4;
    private final double WAITING_SERVO_POSITION = 0.64;
    //    ARM POSITIONS
    private final double ARM_SERVO_LAUNCH_POSITION = 0.18;
    private final double ARM_SERVO_HANG_POSITION = 0.1;
    private final double ARM_SERVO_NORMAL_POSITION = 0.3;
    //    WINCH POSITIONS
    private final int WINCH_HANG_POSITION = 18000;
    private final double WINCH_MOTOR_POWER = 0.9;
    private final int WINCH_DOWN_POSITION = 20;
    private RobotHardware robotHardware;

    //    Which direction the arm is currently going
    public enum LAUNCHSTATE {
        WAITING,
        LAUNCH,
    }

    public enum HANGSTATE {
        HANGING,
        DOWN,
    }

    public enum ARMMOTORSTATE {
        UP,
        DOWN,
        NONE,

    }

    public enum ARMSERVOSTATE {
        HANG,
        LAUNCH,
        NORMAL,
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


    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }


    //    ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(ARMMOTORSTATE armState) {
        if (armState == ARMMOTORSTATE.UP) {
            robotHardware.ArmMotor.setTargetPosition(ARM_UP_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_POWER);

        }
        if (armState == ARMMOTORSTATE.DOWN) {
            robotHardware.ArmMotor.setTargetPosition(ARM_DOWN_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(-ARM_POWER);
        }
        if (armState == ARMMOTORSTATE.NONE) {
            robotHardware.ArmMotor.setPower(0);
            robotHardware.ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void moveArmServo(ARMSERVOSTATE armservostate) {
        if (armservostate == ARMSERVOSTATE.HANG) {
            robotHardware.armServo.setPosition(ARM_SERVO_HANG_POSITION);
        }
        if (armservostate == ARMSERVOSTATE.LAUNCH) {
            robotHardware.armServo.setPosition(ARM_SERVO_LAUNCH_POSITION);
        }
        if (armservostate == ARMSERVOSTATE.NORMAL) {
            robotHardware.armServo.setPosition(ARM_SERVO_NORMAL_POSITION);
        }
    }

    public void hangRobot(HANGSTATE hangstate) {
        if (hangstate == HANGSTATE.HANGING) {
            robotHardware.Hangmotor.setTargetPosition(WINCH_HANG_POSITION);
            robotHardware.Hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.Hangmotor.setPower(WINCH_MOTOR_POWER);
            ((PwmControl)robotHardware.armServo).setPwmDisable();
            robotHardware.ArmMotor.setPower(0);

        }
        if (hangstate == HANGSTATE.DOWN) {
            robotHardware.Hangmotor.setTargetPosition(WINCH_DOWN_POSITION);
            robotHardware.Hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.Hangmotor.setPower(-WINCH_MOTOR_POWER);
        }
    }

    // TODO CODE GRIPPER MOVEMENTS
    public void moveGripper(gripperCurrentState gripperState, GRIPPER_SELECTION gripperSelection) {
        robotHardware.GripperLeft.setDirection(Servo.Direction.REVERSE);
        if (gripperSelection == GRIPPER_SELECTION.BOTH) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperLeft.setPosition(GRIPPER_OPEN_VALUE);
                robotHardware.GripperRight.setPosition(GRIPPER_OPEN_VALUE);
            }
            if (gripperState == gripperCurrentState.CLOSE) {
                robotHardware.GripperLeft.setPosition(GRIPPER_CLOSE_VALUE);
                robotHardware.GripperRight.setPosition(GRIPPER_CLOSE_VALUE);
            }
            return;
        }
        if (gripperSelection == GRIPPER_SELECTION.LEFT) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperLeft.setPosition(GRIPPER_OPEN_VALUE);
            } else {
                robotHardware.GripperLeft.setPosition(GRIPPER_CLOSE_VALUE);
            }
            return;
        }

        if (gripperSelection == GRIPPER_SELECTION.RIGHT) {
            if (gripperState == gripperCurrentState.OPEN) {
                robotHardware.GripperRight.setPosition(GRIPPER_OPEN_VALUE);
            } else {
                robotHardware.GripperRight.setPosition(GRIPPER_CLOSE_VALUE);
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

    public void drive(double axial, double lateral, double yaw, double maxPower) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double denominator = Math.max(Math.abs(lateral) + Math.abs(axial) + Math.abs(yaw), 1);
        robotHardware.Frontleft.setPower(((axial + lateral + yaw) / denominator) * maxPower);
        robotHardware.Backleft.setPower((((axial - lateral) + yaw) / denominator) * maxPower);
        robotHardware.Frontright.setPower((((axial - lateral) - yaw) / denominator) * maxPower);
        robotHardware.Backright.setPower((((axial + lateral) - yaw) / denominator) * maxPower);
    }
}

