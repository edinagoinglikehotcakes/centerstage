package hotcakes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    private OpMode myOpMode;
    public DcMotorEx Frontleft = null;
    public DcMotorEx Backleft = null;
    public DcMotorEx Frontright = null;
    public DcMotorEx Backright = null;
    public DcMotorEx TurnMotor = null;
    public IMU imu = null;
    public DcMotorEx ArmMotor = null;
    public Servo GripperLeft = null;
    public Servo GripperRight = null;
    public Servo ArmServo = null;
    public Servo GripperFlipper = null;

    public RobotHardware(OpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {

//        SERVOS
        ArmServo = myOpMode.hardwareMap.get(Servo.class, "Armservo");
        GripperFlipper = myOpMode.hardwareMap.get(Servo.class,"Gripperflipper");
        GripperRight = myOpMode.hardwareMap.get(Servo.class, "Gripperright");
        GripperLeft = myOpMode.hardwareMap.get(Servo.class, "Gripperleft");


//        MOTORS
        Frontleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontleft");
        Backleft = myOpMode.hardwareMap.get(DcMotorEx.class, "Backleft");
        Frontright = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontright");
        Backright = myOpMode.hardwareMap.get(DcMotorEx.class, "Backright");
        TurnMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Turnmotor");
        ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "Armmotor");

        ArmMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TurnMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        TurnMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //TODO Default to built-in PIDF. Tune this if needed.
        TurnMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,  TurnMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        ArmMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ArmMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));

        //Motors direction
        Frontleft.setDirection(DcMotorEx.Direction.FORWARD);
        Backleft.setDirection(DcMotorEx.Direction.REVERSE);
        Frontright.setDirection(DcMotorEx.Direction.REVERSE);
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
