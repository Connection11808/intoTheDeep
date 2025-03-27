package org.firstinspires.ftc.teamcode.OPPmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.lockToPos;
import org.firstinspires.ftc.teamcode.state_machine.AxisStateMachine;
import org.firstinspires.ftc.teamcode.sys.IntakeArm;
import org.firstinspires.ftc.teamcode.sys.Lift;
import org.firstinspires.ftc.teamcode.sys.OutTakeArm;

import java.util.List;

@TeleOp(name = "GLOMO🥵", group = "drive")
public class POV_DRIVE extends LinearOpMode {
    private ElapsedTime grip_timer = new ElapsedTime();
    private boolean intakeAxisRunningToPos = false;
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
    private double outTakeGripClose = 0.58;
    private double outTakePitchBasket = 0.95;
    private double PitchGetSpec = 0.00;
    private double PitchreadyToSpec = 0.39;
    private double PitchPointWithSpec = 0.91;
    private double Pitchtransfer = 0.24;
    private double outTakeAxisGetSpec = 1;


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

    public enum servoState {
        OPEN(0.66),
        CLOSE(0.93);

        Double value;

        private servoState(Double value)
        {
            this.value = value;
        }
    }

    private double servoPos;// Start at open position
    private Servo pitchServo = null;
    private double PitchPos = 0.765;


    private Servo gripServo = null;
    private Servo sunServo = null;
    private Servo planetServo = null;
    private double grip_pos = 0;
    private double planet_pos = 0;
    private double gripClose = 0.01;
    private double planetUP = 0.359;
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
    private boolean pointWithSpec = false;

    private IntakeArm intakeArm = null;

    private OutTakeArm outTakeArm = null;
    private AxisStateMachine takeSpecHPStateMachine = null;


    private Lift lift = null;
    private double RETRIVE_POWER = 0;
    private boolean grip_previous_value = false;
    private boolean pitchPreVal = false;
    private boolean sunbutton = false;
    private boolean sunlastVal = false;

    private double DRIVE_FACTOR = 1;

    private boolean PREV_VALUE_AXIS_STATE_MACHINE = false;
    private boolean OLV = false;
    private boolean PREV_VALUE_retrieval_open = false;
    private boolean PREV_VALUE_retrieval_close = false;
    private boolean PREV_VALUE_GO_FROM_CHAMBER = false;
    private boolean chamberState = false;
    private double servo_grip_close_position = 0.05;
    private double servo_grip_small_open_position = 0.13;

    private double servo_grip_big_open_position = 0.19;
    String TAG_RR = "rr";
    Pose2d PosToLock;
    Pose2d CLICK_POSITION;
    public static double xyP = 0.02;
    public static double headingP = 0.96;
    private SampleMecanumDrive drive_to_pos;
    private DcMotor retriveal = null;
    private ElapsedTime leftButtonPressTimer = new ElapsedTime();
    private ElapsedTime rightButtonPressTimer = new ElapsedTime();

    private boolean leftButtonHeld = false;
    private boolean rightButtonHeld = false;
    private double retrivealPower = 0;
    private boolean CANGOBACK = false;
    private boolean CANGOFORWARD = true;

    private final boolean GIL_IS_HOMO = true;
    private  com.qualcomm.robotcore.hardware.CRServo Servoleft = null;
    private com.qualcomm.robotcore.hardware.CRServo Servoright = null;

    //constant,this should always be "true", or the code will stop working🤷‍♂️
    Pose2d GoToBasket;

    @Override
    public void runOpMode() throws InterruptedException
    {

        drive_to_pos = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-57, -57, Math.toRadians(90));

        drive_to_pos.setPoseEstimate(startPose);
        Pose2d BasketPos = new Pose2d(-59, -59, Math.toRadians(70));
        GoToBasket = BasketPos;
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive = new Motor(hardwareMap, "left_front");
        leftBackDrive = new Motor(hardwareMap, "left_back");
        rightFrontDrive = new Motor(hardwareMap, "right_front");
        rightBackDrive = new Motor(hardwareMap, "right_back");


        gripServo = hardwareMap.get(Servo.class, "grip_Servo");
        sunServo = hardwareMap.get(Servo.class, "sun_Servo");
        planetServo = hardwareMap.get(Servo.class, "planet_Servo");
//        Servoright = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, " servoright");
//        Servoleft = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, " servoleft");
//        Servoleft.setDirection(Servo.Direction.);



        grip_pos = 0.38;
        sunServo.setPosition(0.03);
        gripServo.setDirection(Servo.Direction.REVERSE);
        planet_pos = planetUP;
        gripServo.setPosition(grip_pos);
        planetServo.setPosition(planet_pos);


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        axis = hardwareMap.get(DcMotor.class, "axis");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        retriveal = hardwareMap.get(DcMotor.class, "retriveal");
        retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retriveal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axis.setDirection(DcMotorSimple.Direction.FORWARD);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        axis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


//        servoPos = servoState.CLOSE.value;
//        // Initialize the hardware variables.
//        ServoretrievalA = hardwareMap.get(Servo.class, "ServoA");
//        // Inverse control for A
//        ServoretrievalB = hardwareMap.get(Servo.class, "ServoB");
//        ServoretrievalA.setDirection(Servo.Direction.REVERSE);
//        ServoretrievalA.setPosition(servoPos);
//        ServoretrievalB.setPosition(servoPos);
        pitchServo = hardwareMap.get(Servo.class, "pitch_Servo");
        pitchServo.setPosition(PitchPos);

        outakeServo = hardwareMap.get(Servo.class, "outake_Servo");
        outakeAxisServo = hardwareMap.get(Servo.class, "outakeAxis_Servo");
        liftPitchServo = hardwareMap.get(Servo.class, "liftPitch_Servo");


        outakeAxisServo.setDirection(Servo.Direction.FORWARD);
        // Start with all servos at position 0
        outake_pos = outTakeGripClose;

        liftPitch_pos = PitchGetSpec;


        // Set the initial positions of the servos
        outakeServo.setPosition(outake_pos);
        outakeAxisServo.setPosition(1);
        liftPitchServo.setPosition(liftPitch_pos);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setInverted(true);
        leftBackDrive.setInverted(true);
        rightFrontDrive.setInverted(false);
        rightBackDrive.setInverted(false);


        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));

        MecanumDrive drive = new MecanumDrive(
                false,
                leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive
        );
        imu.resetYaw();


        intakeArm = new IntakeArm(
                pitchServo,
                planetServo,
                sunServo,
                gripServo,
                retriveal);


        outTakeArm = new OutTakeArm(
                outakeServo,
                outakeAxisServo,
                liftPitchServo);

        lift = new Lift(
                leftLift,
                rightLift,
                axis);

        takeSpecHPStateMachine = new AxisStateMachine(
                lift,
                outTakeArm);
        takeSpecHPStateMachine.stop();

        boolean liftManualMode = false;
        boolean liftManualModePrev = false;
        boolean retrievalRunningToPos = false;
        double axisPower = 0;

        double axisSetpoint = axis.getCurrentPosition();
        double liftSetpoint = leftLift.getCurrentPosition();
        lift.setSetpoints(liftSetpoint, axisSetpoint);



        waitForStart();
//        if(!GIL_IS_HOMO)
//        {
//            throw new RuntimeException("Gil wasn't homo, whomp whomp🤷‍♂️");
//        }
        while (opModeIsActive() )
        {

            if (axis.getCurrentPosition() <= -800)
            {
                axisPower = Math.abs(gamepad2.left_stick_y);
            } else
            {
                axisPower = gamepad2.left_stick_y;
            }
            if (Math.abs(axisPower) < 0.1)
            {
                axisPower = 0;
            }

            if (gamepad1.left_stick_button )
            {
                retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                retrivealPower = -0.6;

            } else if (gamepad1.right_stick_button )
            {
                retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                retrivealPower = 0.6;

            } else
            {
                retrivealPower = 0;
            }



            double liftPower = (gamepad2.left_trigger - gamepad2.right_trigger);
            double axial = -gamepad1.left_stick_y * DRIVE_FACTOR;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * DRIVE_FACTOR;
            double yaw = gamepad1.right_stick_x * DRIVE_FACTOR;


            if (gamepad1.options)
            {
                AreUsingFieldCentric = true;
            }
            if (gamepad1.share)
            {
                AreUsingFieldCentric = false;
            }

            if (gamepad1.dpad_left && lateral <=0.05 && yaw <=0.05 && axial <=0.05)
            {
                LockTo(GoToBasket);
                drive_to_pos.update();

            } else if (AreUsingFieldCentric)
            {
                drive.driveFieldCentric(
                        lateral * SPEED_C, axial * SPEED_C, yaw * SPEED_C, imu.getRobotYawPitchRollAngles().getYaw());
            }
            else
            {
                drive.driveRobotCentric(
                        lateral * SPEED_C, axial * SPEED_C, yaw * SPEED_C);
            }

            drive_to_pos.update();

            if (gamepad1.left_trigger >= 0.1)//>= 0.1
            {
                DRIVE_FACTOR = 1;
            } else
            {
                DRIVE_FACTOR = 0.6;
            }


            if (gamepad1.dpad_down && !sunlastVal)
            {
                if (sunbutton)
                {
                    intakeArm.setClawStraight();
                } else
                {
                    intakeArm.setClawSide();
                }
                sunbutton = !sunbutton;
            }
            sunlastVal = gamepad1.dpad_down;

            if (gamepad1.a && !pitchPreVal)
            {
                if (buttonState)
                {
                    intakeArm.goUp();
                } else
                {
                    intakeArm.goDownTeleop();
                    grip_timer.reset();
                    intakeAxisRunningToPos = true;
                }
                buttonState = !buttonState;
            }
            pitchPreVal = gamepad1.a;

            if (intakeAxisRunningToPos && grip_timer.milliseconds() >= 200)
            {
                intakeArm.openGrip();
                intakeAxisRunningToPos = false;
            }

            if (gamepad2.y)
            {
                takeSpecHPStateMachine.stop();
                liftManualMode = false;
                //                liftPitch_pos = Pitchtransfer;
                outTakeArm.TRANSFER_TELEOP_TELEOP();
                outTakeArm.OUT_TAKE_GRIP_OPEN();
                lift.TRANSFER_TELEOP();//                outakeAxis_pos = outTakeAxisPointWithSpec; //transfer
            }

            if (gamepad2.x && !PREV_VALUE_AXIS_STATE_MACHINE)
            {
                lift.GET_FROM_HP_DOWN();
                outTakeArm.GET_SPEC_FROM_H();

                liftManualMode = false;
            }
            PREV_VALUE_AXIS_STATE_MACHINE = gamepad2.x;
            if (gamepad2.dpad_right)
            {
                takeSpecHPStateMachine.stop();
                lift.PLACE_SPEC();
                liftManualMode = false;
                outTakeArm.PLACE_SPEC_FINISH();
            } else if (gamepad2.dpad_left)
            {
                lift.SPEC_FINISH();
                liftManualMode = false;
            }

            if (gamepad2.dpad_up)
            {
                takeSpecHPStateMachine.stop();
                lift.BASKET_8_POINT();
                outTakeArm.BASKET_8_POINT();
                //liftPitch_pos = outTakePitchBasket; //high basket//outakeAxis_pos = outTakeAxisBasket;
                liftManualMode = false;
            }

            if(gamepad1.x)
            {
             intakeArm.camera_goDown();
            }
//
//            liftController.setSetPoint(liftSetpoint);
//            axisController.setSetPoint(axisSetpoint);
//
//            double liftPowerPID = liftController.calculate(rightLift.getCurrentPosition());
//            rightLift.setPower(liftPowerPID);
//            leftLift.setPower(liftPowerPID);
//
//            axis.setPower(axisController.calculate(axis.getCurrentPosition()));
            if (liftPower != 0 || axisPower != 0)
            {
                takeSpecHPStateMachine.stop();
                liftManualMode = true;
            }
            if (liftManualMode)
            {
                if (!liftManualModePrev)
                {
                    liftSetpoint = lift.get_lift_position();
                    axisSetpoint = lift.get_axis_position();
                }
                liftSetpoint += liftPower * 80;
                axisSetpoint += axisPower * 40;
                lift.setSetpoints(liftSetpoint, axisSetpoint);
            }
            liftManualModePrev = liftManualMode;

            lift.runAxisToSetpoint();
            lift.runLiftToSetpoint();


            if (retrievalRunningToPos)
            {
                StopMotorIfReachedPosition(retriveal);

            } else
            {
                retriveal.setPower(retrivealPower);
            }
            if(gamepad1.right_stick_button ||gamepad1.left_stick_button){
                retrievalRunningToPos = false;

            }




            if (gamepad1.right_bumper && !grip_previous_value)
            {
                if (grip_button)
                {
                    intakeArm.closeGrip();
                } else
                {
                    intakeArm.openGrip();
                }
                grip_button = !grip_button;
            }
            grip_previous_value = gamepad1.right_bumper;
            if (gamepad1.y )
            {
                    intakeArm.TELEOP_INTAKE_MID();
            }

            if (gamepad2.left_bumper && !OLV)
            {
                if (outake_button)
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                } else
                {
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                }
                outake_button = !outake_button;
            }
            OLV = gamepad2.left_bumper;


//            if (gamepad1.left_stick_button && !PREV_VALUE_retrieval_open)
//            {
//
//            }
//            PREV_VALUE_retrieval_open = gamepad1.left_stick_button;

            if (gamepad1.right_trigger >= 0.1 && !PREV_VALUE_retrieval_close)
            {
                intakeArm.retrievalGoBack();
                retrievalRunningToPos = true;
            }
            PREV_VALUE_retrieval_close = gamepad1.right_trigger >= 0.1;

//            ServoretrievalB.setPosition(servoPos);
//            ServoretrievalA.setPosition(servoPos);

            takeSpecHPStateMachine.update();

            // Show the updated servo positions and elapsed game time
//            telemetry.addData("hand_POS_A", ServoretrievalA.getPosition());
//            telemetry.addData("hand_POS_B", ServoretrievalB.getPosition());

            telemetry.addData("outakeaxis", outakeAxisServo.getPosition());
            telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("SPEED at the moment", SPEED_C);
            telemetry.addData("SPEED count", SPEED_COUNT);

            telemetry.update();

            FtcDashboard.getInstance().getTelemetry().addData("axis Motor Position", axis.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("retriveal Motor Position", retriveal.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("lift left POSITION", leftLift.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("lift right POSITION", rightLift.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().addData("axis power", axis.getPower());
            FtcDashboard.getInstance().getTelemetry().addData("x button state", gamepad1.x);
            FtcDashboard.getInstance().getTelemetry().addData("lift setpoint", liftSetpoint);
            FtcDashboard.getInstance().getTelemetry().addData("axis setpoint", axisSetpoint);

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }

    private boolean setMotorToPos(DcMotor motor, int pos, double power)
    {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        return true;
    }

    private boolean StopMotorIfReachedPosition(DcMotor motor)
    {

        if (!motor.isBusy() && motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
        {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        return true;
    }

    public void LockTo(Pose2d targetPos)
    {
        Pose2d currentPos = drive_to_pos.getPoseEstimate();
        Pose2d difference = targetPos.minus(currentPos);
        Vector2d xy = difference.vec().rotated(-currentPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading() - Angle.normDelta(currentPos.getHeading()));
        Log.d(TAG_RR, "difference" + xy.component1() + " " + xy.component2());
        Log.d(TAG_RR, "heading" + heading);
        if ((Math.abs(xy.component1()) > 0.2) || (Math.abs(xy.component2()) > 0.2) || (Math.abs(heading) > 0.2))
        {
            drive_to_pos.setWeightedDrivePower(new Pose2d(xy.times(xyP), heading * headingP));

        }
    }
}

