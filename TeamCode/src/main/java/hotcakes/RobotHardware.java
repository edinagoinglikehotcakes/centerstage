package hotcakes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotHardware {
    private OpMode myOpMode = null;
    public DcMotorEx Frontleft = null;
    public DcMotorEx Backleft = null;
    public DcMotorEx Frontright = null;
    public DcMotorEx Backright = null;
    public DcMotorEx TurnMotor = null;
    public DcMotorEx ArmMotor = null;

    public RobotHardware (OpMode opMode) {myOpMode = opMode;}

    public void init() {
        Frontleft  = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontleft");
        Backleft  = myOpMode.hardwareMap.get(DcMotorEx.class, "Backleft");
        Frontright = myOpMode.hardwareMap.get(DcMotorEx.class, "Frontright");
        Backright = myOpMode.hardwareMap.get(DcMotorEx.class, "Backright");
        TurnMotor = myOpMode.hardwareMap.get(DcMotorEx.class,"Turnmotor");
        ArmMotor = myOpMode.hardwareMap.get(DcMotorEx.class,"Armmotor");

        //Motor direction

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
