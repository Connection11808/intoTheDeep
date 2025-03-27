package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "LIFT_TEST", group = "drive")
@Disabled
public class TEST extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private DcMotor axis = null;
    private Motor leftBackDrive = null;
    private Motor rightFrontDrive = null;
    private Motor rightBackDrive = null;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;
    private boolean SPEED_STOP = false;
    private double outTakeGripOpen = 0.18;
    private double outTakeGripClose = 0.5;
    private double outTakePitchBasket = 0.95;
    private double PitchGetSpec = 0.0;
    private double PitchreadyToSpec = 0.61;
    private double PitchPointWithSpec = 0.91;
    private double Pitchtransfer = 0.24;
    private double outTakeAxisGetSpec = 0.28;
    private double outTakeAxisBasket = 0.4;
    private double outTakeAxisPointWithSpec = 0.9;


    private int SPEED_C = 1; // NOTE: one because we are dividing the power by the SPEED_C
    private int SPEED_COUNT = 0;
    private boolean toggle = false;

    private IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;


    double initYaw;
    double adjustedYaw;
    private Servo ServoretrievalA = null;
    private Servo ServoretrievalB = null;
    private boolean done = false;
//    private double openPos = 1;
//    private double closePos = 0.73;

    private enum servoState {
        OPEN(0.4),
        CLOSE(0.8);

        Double value;

        private servoState(Double value) {
            this.value = value;
        }
    }

    private double servoPos;// Start at open position
    private Servo pitchServo = null;
    private double PitchPos = 0.79;

    private double pitchUp = 0.79;
    private double pitchDown = 0.7;
    private Servo gripServo = null;
    private Servo sunServo = null;
    private Servo planetServo = null;
    private double grip_pos = 0;
    private double sun_pos = 0;
    private double planet_pos = 0;
    private double gripClose = 0.01;
    private double gripOpen = 0.07;
    private double sunStraight = 0.058;
    private double sunSide = 0.003;
    private double planetUP = 0.49;
    private double planetDown = 0.69;
    private boolean press = false;

    private boolean PlanetIsDown = false;
    private boolean GripIsClosed = false;
    private boolean PitchIsDown = false;
    private boolean grip_button = false;
    private boolean outake_button = false;
    private DcMotor leftLift, rightLift;
    private List<DcMotor> LIFT;


    private boolean AreUsingFieldCentric = true;
    private boolean buttonState = false;

    private Servo outakeServo = null;
    private Servo outakeAxisServo = null;
    private Servo liftPitchServo = null;

    private double outake_pos = 0;
    private double outakeAxis_pos = 0;
    private double liftPitch_pos = 0;

    private double TARGET_POSITION_SPEED = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        axis = hardwareMap.get(DcMotor.class, "axis");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.x) {
                // Ensure both motors are in RUN_TO_POSITION mode before setting power

                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                leftLift.setTargetPosition(0);
                rightLift.setTargetPosition(0);



                // Set the target position for both motors to 0

//                // Set power to motors to move to target position
//                leftLift.setPower(-1);  // Adjust direction if necessary
//                rightLift.setPower(-1); // Adjust direction if necessary
            }

// After motors finish moving, stop and set to RUN_WITHOUT_ENCODER
//            if (!leftLift.isBusy() && !rightLift.isBusy()) {
//                leftLift.setPower(0);
//                rightLift.setPower(0);
//
//                // Set motors to RUN_WITHOUT_ENCODER after they finish moving
//                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }


            FtcDashboard.getInstance().getTelemetry().addData("axis Motor Position", axis.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("lift left POSITION", leftLift.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("lift right POSITION", rightLift.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("axis power", axis.getPower());

            FtcDashboard.getInstance().getTelemetry().update();
        }



    }
}

