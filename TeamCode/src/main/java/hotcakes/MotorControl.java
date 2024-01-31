package hotcakes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorControl {
    //    set limits
    private final int ARM_LIMIT = -2180;
    private final double ARM_POWER = 0.5;

    //    TODO CHANGE SOME OF THESE VALUES ACCORDING TO TUNING
    private final int ARM_PICKUP_TARGET_POSITION = -470;
    private final int ARM_DOWN_TARGET_POSITION = -40;
    private final int ARM_BACKDROP_TARGET_POSITION = -1050;
    private final double DEFAULT_LAUNCH_RANGE = 72;
    private final double SERVO_FLIPPER_DROP_POSITION = 0.45;
    private final double SERVO_FLIPPER_DRIVE_POSITION = 0.4;
    private final double SERVO_FLIPPER_PICKUP_POSITION = 0.575;
    //    LAUNCH SERVO
    private final double LAUNCHING_SERVO_POSITION = 0.4;
    private final double WAITING_SERVO_POSITION = 0.64;
    //    ARM POSITIONS

    private final double ARM_SERVO_Drive_POSITION = 0.2;
    private final double ARM_SERVO_Pickup_POSITION = 0.03;
    private final double ARM_SERVO_Backdrop_POSITION = 0.35;
    //    WINCH POSITIONS
    private final int WINCH_HANG_POSITION = 9000;
    private final double WINCH_MOTOR_POWER = 0.9;
    private final int WINCH_DOWN_POSITION = 3500;
    private RobotHardware robotHardware;
    private PixelStackAprilTags pixelStacklAprilTags = null;
    private final double DEFAULT_LAUNCH_ANGLE = .4;

    private final double TAG_RANGE = 72;

    //    Which direction the arm is currently going
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


    //    ARM MOVEMENT FOR UP AND DOWN
    public void mobilizeArm(ArmExtension armState) {
        if (armState == ArmExtension.PICKUP) {
            robotHardware.ArmMotor.setTargetPosition(ARM_PICKUP_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(ARM_POWER);
            if (robotHardware.GripperAngle.getPosition() != 0.56) {
                robotHardware.GripperAngle.setPosition(0.575);
            }

        }
        if (armState == ArmExtension.RETRACT) {
            robotHardware.ArmMotor.setTargetPosition(ARM_DOWN_TARGET_POSITION);
            robotHardware.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.ArmMotor.setPower(-ARM_POWER);
            robotHardware.GripperAngle.setPosition(0.4);
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

    public void changeArmAngle(ArmAngle armservostate) {
        if (armservostate == ArmAngle.PICKUP) {
            robotHardware.GripperAngle.setPosition(0.7);
            robotHardware.GripperAngle.setPosition(SERVO_FLIPPER_PICKUP_POSITION);
            robotHardware.ArmAngle.setPosition(ARM_SERVO_Pickup_POSITION);
        }
        if (armservostate == ArmAngle.DRIVE) {
            robotHardware.ArmAngle.setPosition(ARM_SERVO_Drive_POSITION);
            robotHardware.GripperAngle.setPosition(0.9);
        }
        if (armservostate == ArmAngle.BACKDROP) {
            robotHardware.ArmAngle.setPosition(ARM_SERVO_Backdrop_POSITION);
            robotHardware.GripperAngle.setPosition(0.56);
        }
    }

    public void hangRobot(HangState hangstate) {
        if (hangstate == HangState.HANGING) {
            robotHardware.HangMotor.setTargetPosition(WINCH_HANG_POSITION);
            robotHardware.HangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotHardware.HangMotor.setPower(WINCH_MOTOR_POWER);
            ((PwmControl) robotHardware.ArmAngle).setPwmDisable();
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
                robotHardware.GripperLeft.setPosition(0.75);
                robotHardware.GripperRight.setPosition(0.1);
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
            return;
        }
    }

    public void launchPlane(LaunchState launchState) {
        if (launchState == LaunchState.LAUNCH) {
            launch();
        }

        if (launchState == LaunchState.WAITING) {
            robotHardware.LaunchAngle.setPosition(DEFAULT_LAUNCH_ANGLE);
            robotHardware.DroneLaunch.setPosition(WAITING_SERVO_POSITION);
        }
    }

    public void flipGripper(GripperAngle gripperAngle) {
        if (gripperAngle == GripperAngle.BACKSTAGE) {
            robotHardware.GripperAngle.setPosition(SERVO_FLIPPER_DROP_POSITION);
        }
        if (gripperAngle == GripperAngle.PICKUP) {
            robotHardware.GripperAngle.setPosition(SERVO_FLIPPER_PICKUP_POSITION);
        }
    }
    public void changeLaunchAngle(LaunchAngle launchAngle) {
        if (launchAngle == LaunchAngle.LAUNCH) {
            robotHardware.LaunchAngle.setPosition(0.3);
        }
        if (launchAngle == LaunchAngle.WAITING) {
            robotHardware.LaunchAngle.setPosition(0);
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
        while (timer.milliseconds() <= 500) {
        }

        // Launch!
        robotHardware.DroneLaunch.setPosition(LAUNCHING_SERVO_POSITION);
    }
    private double getLaunchPosition(double range) {
        return ((1 - ((range - TAG_RANGE) / (TAG_RANGE)) *
                (LAUNCHING_SERVO_POSITION - WAITING_SERVO_POSITION)) + WAITING_SERVO_POSITION);
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

