package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.state_machine.AxisStateMachine;
import org.firstinspires.ftc.teamcode.sys.IntakeArm;
import org.firstinspires.ftc.teamcode.sys.Lift;
import org.firstinspires.ftc.teamcode.sys.OutTakeArm;
import org.firstinspires.ftc.teamcode.sys.RectDrawer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(group = "RED_Specimen")
public class CameraAutoNoCamera extends LinearOpMode {
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
    private double sample_xPosition;
    private double sample_yPosition;

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
    private WebcamName cameraName;
    private Servo outakeServo = null;
    private Servo outakeAxisServo = null;
    private Servo liftPitchServo = null;
    private final Size CAMERA_RESOLUTION = new Size(640, 480);
    private double outake_pos = 0;
    private double outakeAxis_pos = 0;
    private double liftPitch_pos = 0;

    private double TARGET_POSITION_SPEED = 0.0;
    private boolean pointWithSpec = false;

    private IntakeArm intakeArm = null;

    private OutTakeArm outTakeArm = null;
    private AxisStateMachine takeSpecHPStateMachine = null;
    private DcMotor retriveal = null;


    private Lift lift = null;
    private RectDrawer rectDrawer = null;

    @Override
    public void runOpMode() throws InterruptedException {

        gripServo = hardwareMap.get(Servo.class, "grip_Servo");
        gripServo.setDirection(Servo.Direction.REVERSE);
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

        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        outakeAxisServo.setDirection(Servo.Direction.FORWARD);
        // Start with all servos at position 0
        outake_pos = outTakeGripClose;

        liftPitch_pos = PitchGetSpec;


        // Set the initial positions of the servos
        gripServo.setPosition(1);
        outakeAxisServo.setPosition(1);
        liftPitchServo.setPosition(liftPitch_pos);
        axis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





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
        rectDrawer = new RectDrawer(telemetry);
        takeSpecHPStateMachine.stop();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d StartPos = new Pose2d(-39.8,-62.625, Math.toRadians(90));
        outTakeArm.OUT_TAKE_GRIP_CLOSE();
        intakeArm.openGrip();
        drive.setPoseEstimate(StartPos);
        VisionPortal visionPortal = buildVisionPortal(cameraName);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        TrajectorySequence tjs = drive.trajectorySequenceBuilder(StartPos)
                .addDisplacementMarker(() ->
                {
                    intakeArm.retrievalFront();

                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(40)))
                .waitSeconds(0.4)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                })
                .lineToLinearHeading(new Pose2d(-47.5, -39.9, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goDown();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->
                {
                    intakeArm.closeGrip();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-56.5, -56.5, Math.toRadians(40)))
                .waitSeconds(0.4)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                })
                .lineToLinearHeading(new Pose2d(-58, -39.9, Math.toRadians(88)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goDown();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {
                    intakeArm.closeGrip();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .waitSeconds(1)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();

                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-56.5, -56.5, Math.toRadians(40)))
                .waitSeconds(0.4)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })

                .build();
        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequence(tjs);

        while (!isStopRequested() && opModeIsActive()) {
//            drive.update();
        }
    }
    public VisionPortal buildVisionPortal(WebcamName cameraName)
    {
        rectDrawer = new RectDrawer(telemetry);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)// ADD PROCESSORS HERE
                .addProcessors(rectDrawer)
                .build();

        visionPortal.setProcessorEnabled(rectDrawer, true);  // let processors run asynchronously using camera dat
        return visionPortal;
    }

}
