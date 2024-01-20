package hotcakes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RobotHardware {
    private OpMode myOpMode;
    public DcMotorEx Frontleft = null;
    public DcMotorEx Backleft = null;
    public DcMotorEx Frontright = null;
    public DcMotorEx Backright = null;
    //    public DcMotorEx TurnMotor = null;
    public IMU imu = null;
    public DcMotorEx ArmMotor = null;
    public DcMotorEx HangMotor = null;
    public Servo GripperLeft = null;
    public Servo GripperRight = null;
    public ServoImplEx ArmAngle = null;
    public Servo DroneLaunch = null;
    public Servo LaunchAngle = null;

    public RobotHardware(OpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {

//        SERVOS
        GripperRight = myOpMode.hardwareMap.get(Servo.class, "Gripperright");
        GripperLeft = myOpMode.hardwareMap.get(Servo.class, "Gripperleft");
        DroneLaunch = myOpMode.hardwareMap.get(Servo.class, "launchServo");
        ArmAngle = myOpMode.hardwareMap.get(ServoImplEx.class, "Armservo");
        LaunchAngle = myOpMode.hardwareMap.get(ServoImplEx.class, "LaunchAngle");


//        MOTORS
        Frontleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontleft");
        Backleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Backleft");
        Frontright = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontright");
        Backright = myOpMode.hardwareMap.get(DcMotorEx.class, "Backright");
        ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Armmotor");
        HangMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Hangmotor");

        ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //TODO Default to built-in PIDF. Tune this if needed.
        ArmMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ArmMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        HangMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, HangMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
//        TODO test direction of motors

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

    }
}
