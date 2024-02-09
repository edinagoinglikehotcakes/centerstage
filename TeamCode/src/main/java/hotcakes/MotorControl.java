package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MotorControl {
    //    set limits
    private final double ARM_POWER = 0.5;
    // TODO CHANGE SOME OF THESE VALUES ACCORDING TO TUNING
    private final int ARM_PICKUP_TARGET_POSITION = -480;
    private final int ARM_DOWN_TARGET_POSITION = -40;
    private final int ARM_BACKDROP_TARGET_POSITION = -1050;
    private final double DEFAULT_LAUNCH_RANGE = 72;
    private final double SERVO_FLIPPER_DROP_POSITION = 0.45;
    private final double SERVO_FLIPPER_DRIVE_POSITION = 0.4;
    private final double SERVO_FLIPPER_PICKUP_POSITION = 0.23;
    // LAUNCH SERVO
    private final double LAUNCHING_SERVO_POSITION = 0.35;
    private final double WAITING_SERVO_POSITION = 0.58;
    // ARM POSITIONS
    private final double ARM_ANGLE_MOTOR_POWER = 0.9;
    // WINCH POSITIONS
    private final int WINCH_HANG_POSITION = 9000;
    private final double WINCH_MOTOR_POWER = 0.9;
    private final int WINCH_DOWN_POSITION = 3500;
    private RobotHardware robotHardware;
    private PixelStackAprilTags pixelStacklAprilTags = null;
    private final double DEFAULT_LAUNCH_ANGLE = .4;
    private final double TAG_RANGE = 72;
    // Launch
    private final double LAUNCH_SERVO_ANGLE = 0.5;
    private final double WAITING_SERVO_ANGLE = 0;

    // Which direction the arm is currently going
    public enum LaunchState {
        WAITING,
        LAUNCH,
    }

    public enum HangState {
        HANGING,
        DOWN,
    }

    //TODO Change Code
    public enum ArmExtension {
        PICKUP,
        BACKDROP,
        RETRACT,
        NONE,
    }

    //TODO Change Code
    public enum ArmAngle {
        PICKUP,
        DRIVE,
        BACKDROP,
        MAXANGLE,
    }

    public enum GripperState {
        OPEN,
        CLOSE,
    }

    public enum GripperSelection {
        LEFT,
        RIGHT,
        BOTH,
    }

    public enum GripperAngle {
        PICKUP,
        BACKSTAGE,
    }

    public enum LaunchAngle {
        LAUNCH,
        WAITING,
    }

    public MotorControl(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    // ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(ArmExtension armState) {
        if (armState == ArmExtension.PICKUP) {
            robotHardware.ArmMotor.setTargetPosition(ARM_PICKUP_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_POWER);
            if (robotHardware.GripperAngle.getPosition() != 0.193) {
                robotHardware.GripperAngle.setPosition(0.30);
            }
        }

        if (armState == ArmExtension.RETRACT) {
            robotHardware.ArmMotor.setTargetPosition(ARM_DOWN_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(-ARM_POWER);
            robotHardware.GripperAngle.setPosition(0.13);
        }

        if (armState == ArmExtension.BACKDROP) {
            robotHardware.ArmMotor.setTargetPosition(ARM_BACKDROP_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_POWER);
        }

        if (armState == ArmExtension.NONE) {
            robotHardware.ArmMotor.setPower(0);
            robotHardware.ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void changeArmAngle(ArmAngle armanglestate) {
        if (armanglestate == ArmAngle.PICKUP) {
            robotHardware.GripperAngle.setPosition(0.3);
            robotHardware.ArmAngle.setTargetPosition(35);
            robotHardware.ArmAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmAngle.setPower(0.5);
        }

        if (armanglestate == ArmAngle.DRIVE) {
            robotHardware.ArmAngle.setTargetPosition(600);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_ANGLE_MOTOR_POWER);
            robotHardware.GripperAngle.setPosition(0.34);
        }

        if (armanglestate == ArmAngle.BACKDROP) {
            robotHardware.ArmAngle.setTargetPosition(735);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_ANGLE_MOTOR_POWER);
            robotHardware.GripperAngle.setPosition(0.193);
        }

        if (armanglestate == ArmAngle.MAXANGLE) {
            robotHardware.ArmAngle.setTargetPosition(770);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_ANGLE_MOTOR_POWER);
            robotHardware.GripperAngle.setPosition(0.193);
        }
    }

    public void hangRobot(HangState hangstate) {
        if (hangstate == HangState.HANGING) {
            robotHardware.HangMotor.setTargetPosition(WINCH_HANG_POSITION);
            robotHardware.HangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.HangMotor.setPower(WINCH_MOTOR_POWER);
            robotHardware.ArmMotor.setPower(0);
        }

        if (hangstate == HangState.DOWN) {
            robotHardware.HangMotor.setTargetPosition(WINCH_DOWN_POSITION);
            robotHardware.HangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.HangMotor.setPower(WINCH_MOTOR_POWER);
        }
    }

    // TODO CODE GRIPPER MOVEMENTS
    public void moveGripper(GripperState gripperState, GripperSelection gripperSelection) {
        if (gripperSelection == GripperSelection.BOTH) {
            if (gripperState == GripperState.OPEN) {
                robotHardware.GripperLeft.setPosition(0.78);
                robotHardware.GripperRight.setPosition(0.07);
            }

            if (gripperState == GripperState.CLOSE) {
                robotHardware.GripperLeft.setPosition(0.66);
                robotHardware.GripperRight.setPosition(0.20);
            }

            return;
        }

        if (gripperSelection == GripperSelection.LEFT) {
            if (gripperState == GripperState.OPEN) {
                robotHardware.GripperLeft.setPosition(0.75);
            } else {
                robotHardware.GripperLeft.setPosition(0.66);
            }

            return;
        }

        if (gripperSelection == GripperSelection.RIGHT) {
            if (gripperState == GripperState.OPEN) {
                robotHardware.GripperRight.setPosition(0.1);
            } else {
                robotHardware.GripperRight.setPosition(0.2);
            }
        }
    }

    public void launchPlane(LaunchState launchState) {
        if (launchState == LaunchState.LAUNCH) {
            launch();
        }

        if (launchState == LaunchState.WAITING) {
            robotHardware.LaunchAngle.setPosition(0);
            robotHardware.DroneLaunch.setPosition(WAITING_SERVO_POSITION);
        }
    }

    public void flipGripper(GripperAngle gripperAngle) {
        if (gripperAngle == GripperAngle.BACKSTAGE) {
            robotHardware.GripperAngle.setPosition(0.165);
        }

        if (gripperAngle == GripperAngle.PICKUP) {
            robotHardware.GripperAngle.setPosition(SERVO_FLIPPER_PICKUP_POSITION);
        }
    }

    public void changeLaunchAngle(LaunchAngle launchAngle) {
        if (launchAngle == LaunchAngle.LAUNCH) {
            robotHardware.LaunchAngle.setPosition(LAUNCH_SERVO_ANGLE);
        }

        if (launchAngle == LaunchAngle.WAITING) {
            robotHardware.LaunchAngle.setPosition(WAITING_SERVO_ANGLE);
        }
    }

    private void launch() {
        pixelStacklAprilTags = new PixelStackAprilTags(robotHardware.myOpMode.hardwareMap);
        pixelStacklAprilTags.init();
        AprilTagDetection detectedTag = pixelStacklAprilTags.detectTags();
        double launchRange = detectedTag == null ? DEFAULT_LAUNCH_RANGE : detectedTag.ftcPose.range;
        pixelStacklAprilTags.disableTagProcessing();
        // Set the launch angle
        robotHardware.LaunchAngle.setPosition(getLaunchPosition(launchRange));
        // Wait for the servo to move.
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() <= 4000) {
        }

        // Launch!
        robotHardware.DroneLaunch.setPosition(LAUNCHING_SERVO_POSITION);
    }

    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCH_SERVO_ANGLE)));
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

