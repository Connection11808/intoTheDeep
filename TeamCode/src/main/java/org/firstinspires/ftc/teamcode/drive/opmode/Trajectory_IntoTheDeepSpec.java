package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.state_machine.AxisStateMachine;
import org.firstinspires.ftc.teamcode.sys.IntakeArm;
import org.firstinspires.ftc.teamcode.sys.Lift;
import org.firstinspires.ftc.teamcode.sys.OutTakeArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(group = "RED_Specimen")
@Disabled
public class Trajectory_IntoTheDeepSpec extends LinearOpMode {
    private ElapsedTime grip_timer = new ElapsedTime();
    private boolean intakeAxisRunningToPos = false;
    private Motor leftFrontDrive = null;
    private DcMotor axis = null;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightFrontPower = 0;
    private double rightBackPower = 0;
    private boolean SPEED_STOP = false;
    private double outTakeGripOpen = 0.18;
    private double outTakeGripClose = 0.52;
    private double outTakePitchBasket = 0.95;
    private double PitchGetSpec = 1;
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
        OPEN(0.69),
        CLOSE(0.93);

        Double value;

        private servoState(Double value)
        {
            this.value = value;
        }
    }

    private double servoPos;// Start at open position
    private Servo pitchServo = null;
    private double PitchPos = 0.79;


    private Servo gripServo = null;
    private Servo sunServo = null;
    private Servo planetServo = null;
    private double grip_pos = 0;
    private double planet_pos = 0;
    private double gripClose = 0.01;
    private double planetUP = 0.49;
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
    private DcMotor retriveal = null;

    private  com.qualcomm.robotcore.hardware.CRServo Servoleft = null;
    private com.qualcomm.robotcore.hardware.CRServo Servoright = null;
    private Lift lift = null;

    @Override
    public void runOpMode() throws InterruptedException {

        gripServo = hardwareMap.get(Servo.class, "grip_Servo");
        sunServo = hardwareMap.get(Servo.class, "sun_Servo");
        planetServo = hardwareMap.get(Servo.class, "planet_Servo");



        planet_pos = planetUP;
        planetServo.setPosition(planet_pos);


        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        axis = hardwareMap.get(DcMotor.class, "axis");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        retriveal = hardwareMap.get(DcMotor.class, "retriveal");
        retriveal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axis.setDirection(DcMotorSimple.Direction.FORWARD);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axis.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        servoPos = servoState.CLOSE.value;
        // Initialize the hardware variables.
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
        Servoright = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, " servoright");
        Servoleft = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, " servoleft");
        Servoleft.setDirection(DcMotorSimple.Direction.REVERSE);




        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d StartPos = new Pose2d(6.875,-62.625, Math.toRadians(90));
        outTakeArm.OUT_TAKE_GRIP_CLOSE();
        intakeArm.closeGrip();
        drive.setPoseEstimate(StartPos);
        intakeArm.TELEOP_INTAKE_MID();
        TrajectorySequence tjs = drive.trajectorySequenceBuilder(StartPos)
                .addTemporalMarker(() ->
                {
                    outTakeArm.GET_SPEC_FROM_H();
                    lift.PLACE_SPEC_AUTO();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->
                {
                    outTakeArm.PLACE_SPEC();
                })
                .waitSeconds(0.2)
                .splineTo(new Vector2d(3.875, -45), Math.toRadians(90))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(3.875, -32, Math.toRadians(90)))
                .waitSeconds(0.3)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .lineToLinearHeading(new Pose2d(13.875, -50, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    lift.GET_FROM_HP_DOWN_AUTO();
                    outTakeArm.GET_SPEC_FROM_H();
                })

                .setReversed(false)
                .splineTo(new Vector2d(36, -20), Math.toRadians(90))
                .splineTo(new Vector2d(38, -13), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(44, -13, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(44, -50, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(42, -9, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(59, -13, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(61, -52, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(59, -44, Math.toRadians(67)))

                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                    intakeArm.goDown();
                    intakeArm.openGrip();
                    servoPos = Trajectory_IntoTheDeep.servoState.OPEN.value;
                    intakeArm.retrievalFrontAuto3();
//                    intakeArm.TURN_AUTO_1();
                })
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {

                    intakeArm.closeGrip();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {

                    servoPos = Trajectory_IntoTheDeep.servoState.CLOSE.value;
                    intakeArm.retrievalBack();
                    intakeArm.goUp();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->
                        outTakeArm.OUT_TAKE_GRIP_CLOSE())
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() ->
                {
                    lift.GET_FROM_HP_DOWN_AUTO();
                    outTakeArm.GET_SPEC_FROM_H();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .lineToLinearHeading(new Pose2d(40, -60, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .addTemporalMarker(() ->
                {
                    outTakeArm.GET_SPEC_FROM_H();
                    lift.PLACE_SPEC_AUTO();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->
                {
                    outTakeArm.PLACE_SPEC();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(3.875, -45, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(3.875, -32, Math.toRadians(90)))
                .waitSeconds(0.3)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .lineToLinearHeading(new Pose2d(13.875, -50, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    lift.GET_FROM_HP_DOWN_AUTO();
                    outTakeArm.GET_SPEC_FROM_H();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .build();
        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequence(tjs);

        while (!isStopRequested() && opModeIsActive()) {
//            drive.update();

//            takeSpecHPStateMachine.update();
        }
    }

}
