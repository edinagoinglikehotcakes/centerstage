package hotcakes;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public OpMode myOpMode;
    public IMU imu;
    public DcMotorEx Frontleft = null;
    public DcMotorEx Backleft = null;
    public DcMotorEx Frontright = null;
    public DcMotorEx Backright = null;
    public DcMotorEx ArmMotor = null;
    public DcMotorEx HangMotor = null;
    public DcMotorEx ArmAngle = null;
    public Servo GripperLeft = null;
    public Servo GripperRight = null;
    public Servo DroneLaunch = null;
    public Servo LaunchAngle = null;
    public Servo GripperAngle = null;
    public HardwareMap hardwareMap;

    public RobotHardware(OpMode opMode) {
        myOpMode = opMode;
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // SERVOS
        GripperRight = myOpMode.hardwareMap.get(Servo.class, "gripperright");
        GripperLeft = myOpMode.hardwareMap.get(Servo.class, "gripperleft");
        GripperAngle = myOpMode.hardwareMap.get(Servo.class, "gripperangle");
        DroneLaunch = myOpMode.hardwareMap.get(Servo.class, "launchservo");
        LaunchAngle = myOpMode.hardwareMap.get(Servo.class, "launchangle");

        // MOTORS
        Frontleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontleft");
        Backleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Backleft");
        Frontright = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontright");
        Backright = myOpMode.hardwareMap.get(DcMotorEx.class, "Backright");
        ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Armmotor");
        HangMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Hangmotor");
        ArmAngle = myOpMode.hardwareMap.get(DcMotorEx.class, "Armangle");

        ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmAngle.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //TODO Default to built-in PIDF. Tune this if needed.
        ArmMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ArmMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        HangMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, HangMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        ArmAngle.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ArmAngle.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));

        Frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        Backleft.setDirection(DcMotorEx.Direction.REVERSE);
        Frontright.setDirection(DcMotorEx.Direction.FORWARD);
        Backright.setDirection(DcMotorEx.Direction.FORWARD);

        Frontleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Backleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Frontleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Backleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Frontright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Backright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Armmotor");
        HangMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Hangmotor");

        ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO Default to built-in PIDF. Tune this if needed.
        ArmMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ArmMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        HangMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, HangMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        // These motors are used in RUN_TO_POSITION.
        ArmMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, ArmMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        HangMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, HangMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
    }
}
