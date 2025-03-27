package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
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

@Autonomous(group = "RED_Specimen", name = "GLOMO_AUTO🥵")
public class CameraAuto extends LinearOpMode {
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
    SampleMecanumDrive drive = null;

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
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.camera_goDown();
                })
                .lineToLinearHeading(new Pose2d(-47.5, -39.9, Math.toRadians(90)))
                .build();


        TrajectorySequence tjs2 = drive.trajectorySequenceBuilder(new Pose2d(-48.4, -39.9, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();

                })
                .waitSeconds(0.6)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(40)))
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                    intakeArm.camera_goDown();
                })

                .build();



        TrajectorySequence tjs3 = drive.trajectorySequenceBuilder(new Pose2d(-56, -56, Math.toRadians(40)))
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                })
                .lineToLinearHeading(new Pose2d(-58.0, -39.9, Math.toRadians(90)))
                .build();

        TrajectorySequence tjs4 = drive.trajectorySequenceBuilder(new Pose2d(-59, -39.9, Math.toRadians(90)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(40)))
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                    intakeArm.camera_goDown();
                })
                .lineToLinearHeading(new Pose2d(-50, -26, Math.toRadians(180)))
                .build();
        TrajectorySequence tjs5 = drive.trajectorySequenceBuilder(new Pose2d(-51, -24, Math.toRadians(180)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();

                })
                .waitSeconds(0.9)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .lineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(40)))
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .build();
        TrajectoryVelocityConstraint constraintSUB = new TranslationalVelocityConstraint(80);
        TrajectorySequence tjs6 = drive.trajectorySequenceBuilder(new Pose2d(-56, -56, Math.toRadians(40)))
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.TRANSFER_TELEOP();
                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retSub();
                    intakeArm.camera_goDown();
                })
                .setVelConstraint(constraintSUB)
                .splineTo(new Vector2d(-18, -7), Math.toRadians(0))
                .build();

        TrajectorySequence tjs7 = drive.trajectorySequenceBuilder(new Pose2d(-18, -7, Math.toRadians(0)))
                .setVelConstraint(constraintSUB)
                .lineToLinearHeading(new Pose2d(-21, -7, Math.toRadians(0)))
                .addTemporalMarker(() ->
                {
                    intakeArm.goUp();
                    intakeArm.retrievalBack();

                })
                .waitSeconds(0.6)
                .addTemporalMarker(() ->
                {
                    outTakeArm.OUT_TAKE_GRIP_CLOSE();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {
                    intakeArm.openGrip();
                    lift.BASKET_8_POINT_AUTO();
                    outTakeArm.BASKET_8_POINT();
                })
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, -55, Math.toRadians(35)), Math.toRadians(215))
                .waitSeconds(0.1)
                .addTemporalMarker(() ->
                {

                    outTakeArm.OUT_TAKE_GRIP_OPEN();
                    intakeArm.retrievalBack();
                })
                .build();
        TrajectorySequence tjs8 = drive.trajectorySequenceBuilder(new Pose2d(-56, -56, Math.toRadians(35)))
                .addTemporalMarker(() ->
                {
                    lift.TRANSFER_AUTO();
                    outTakeArm.GET_SPEC_FROM_H();
                })
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(90)))
                .build();


        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequence(tjs);
        sleep(300);
        moveRobotToSample(new Pose2d(0,0,Math.toRadians(0)), 0.5);
        sleep(500);
        intakeArm.closeGrip();
        sleep(100);
        drive.followTrajectorySequence(tjs2);
        drive.followTrajectorySequence(tjs3);
        sleep(300);
        moveRobotToSample(new Pose2d(0,0,Math.toRadians(0)), 0.5);
        sleep(500);
        intakeArm.closeGrip();
        sleep(200);
        drive.followTrajectorySequence(tjs4);
        sleep(300);
        moveRobotToSample(new Pose2d(0,0,Math.toRadians(0)), 0.5);
        sleep(500);
        intakeArm.closeGrip();
        sleep(200);
        drive.followTrajectorySequence(tjs5);
        drive.followTrajectorySequence(tjs6);
        sleep(600);
        moveRobotToSampleSUB(new Pose2d(0, 0,Math.toRadians(0)), 0.5);
        sleep(500);
        intakeArm.closeGrip();
        sleep(200);
        drive.followTrajectorySequence(tjs7);
        drive.followTrajectorySequence(tjs8);









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
    public void LockTo(Pose2d targetPos)
    {
        Pose2d currentPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currentPos);
        Vector2d xy = difference.vec().rotated(-currentPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading() - Angle.normDelta(currentPos.getHeading()));
        if ((Math.abs(xy.component1()) > 0.2) || (Math.abs(xy.component2()) > 0.2) || (Math.abs(heading) > 0.2))
        {
            drive.setWeightedDrivePower(new Pose2d(xy, heading));

        }
    }
    public void moveRobotToSampleSUB(Pose2d startPose, double Xerror)
    {
        boolean goDownBool = false;
        double error;
        drive = new SampleMecanumDrive(hardwareMap);
        sleep(200);
        double y = rectDrawer.getReal_y();
        double x = rectDrawer.getRealX();

        TrajectoryVelocityConstraint constraint = new TranslationalVelocityConstraint(30);
        TrajectorySequence trajectorySequenceSTART = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x + 0.8)
                .build();
        TrajectorySequence trajectorySequenceSTART33 = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x - 1.2)
                .build();


        ElapsedTime gripTimer = new ElapsedTime();
        if (Math.abs(rectDrawer.getSampleAngle()) < 45)
        {
            if (y > 0)
            {
                intakeArm.moveRetrievalByInch(y + 1.5 + (450 / 100.66));
            }
            else {intakeArm.moveRetrievalByInch(y + 1 + (450 / 100.66));}
            if (x > 0)
            {
                drive.followTrajectorySequence(trajectorySequenceSTART);
            }
            else {drive.followTrajectorySequence(trajectorySequenceSTART33);}

            error = 0.15;
        }
        else
        {
            intakeArm.moveRetrievalByInch(y + 0.5 + (450 / 100.66));
            drive.followTrajectorySequence(trajectorySequenceSTART);
        }
        sleep(200);
        double new_y = rectDrawer.getReal_y();
        double new_x = rectDrawer.getRealX();
        TrajectoryVelocityConstraint constraintsec = new TranslationalVelocityConstraint(30);
        TrajectorySequence trajectorySequenceSTARTsec = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraintsec)
                .strafeRight(new_x + x + 0.2)
                .build();
        TrajectorySequence trajectorySequenceSTARTsec2 = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraintsec)
                .strafeRight(new_x + x - 0.2)
                .build();
        if (Math.abs(rectDrawer.getSampleAngle()) < 36)
        {
            if (new_y > 0)
            {
                intakeArm.moveRetrievalByInch(new_y + 0.4 + y + 1.5 + (450 / 100.66));
            }
            else {intakeArm.moveRetrievalByInch(new_y + 0.4 + y + 1.5 + (450 / 100.66));}
            double servoAngle = 0.36;
            sunServo.setPosition(servoAngle);
            drive.followTrajectorySequence(trajectorySequenceSTARTsec);
            goDownBool = true;
            error = 0.15;
        }
        else
        {
            double servoAngle = 0.03;
            sunServo.setPosition(servoAngle);
            goDownBool = true;
            error = 0.6;
            intakeArm.moveRetrievalByInch(new_y + 0.5 + y + 0.8 + (450 / 100.66));
            if (x > 0)
            {
                drive.followTrajectorySequence(trajectorySequenceSTARTsec);
            }
            else
            {
                drive.followTrajectorySequence(trajectorySequenceSTARTsec2);
            }
        }


        while (true)
        {
            y = rectDrawer.getReal_y() + 0.4;
            x = rectDrawer.getRealX();
            if (x == 0)
            {
                continue;
            }
            if ((Math.abs(rectDrawer.getRealX()) <= Xerror) && (Math.abs(rectDrawer.getReal_y()) <= error) || (goDownBool))
            {
                intakeArm.goDown();
                break;
            }
            TrajectoryVelocityConstraint constraint2 = new TranslationalVelocityConstraint(5);
            TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                    .setVelConstraint(constraint2)
                    .strafeRight(x)
                    .build();
            drive.followTrajectorySequence(trajectorySequence);

            if (Math.abs(y) <= 0.1)
            {
                intakeArm.stopRetrieval();
            }
            else
            {
                intakeArm.moveRetrievalByInch(y);
            }
            drive.update();
        }
    }
    public void moveRobotToSample(Pose2d startPose, double Xerror)
    {
        boolean goDownBool = false;
        double error;
        drive = new SampleMecanumDrive(hardwareMap);
        sleep(200);
        double y = rectDrawer.getReal_y();
        double x = rectDrawer.getRealX();

        TrajectoryVelocityConstraint constraint = new TranslationalVelocityConstraint(30);
        TrajectorySequence trajectorySequenceSTART = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x + 0.5)
                .build();

        ElapsedTime gripTimer = new ElapsedTime();

        if ((Math.abs(rectDrawer.getSampleAngle()) < 36) && (rectDrawer.getRealX() != 0))
        {
            intakeArm.moveRetrievalByInch(y + 0.2);


            double servoAngle = 0.36;
            sunServo.setPosition(servoAngle);
            if (x != 0)
            {

                drive.followTrajectorySequence(trajectorySequenceSTART);
            }
            
            goDownBool = true;
            error = 0.15;
        }
        else
        {
            double servoAngle = 0.03;
            sunServo.setPosition(servoAngle);
            goDownBool = true;
            error = 0.6;
            
            intakeArm.moveRetrievalByInch(y + 0.5);

            if (x != 0){
                drive.followTrajectorySequence(trajectorySequenceSTART);
            }
        }


        while (true)
        {
            y = rectDrawer.getReal_y() + 0.4;
            x = rectDrawer.getRealX();

            if ((x == 0) && (y == 30))
            {
                continue;
            }
            if ((Math.abs(rectDrawer.getRealX()) <= Xerror) && (Math.abs(rectDrawer.getReal_y()) <= error) || (goDownBool))
            {
                intakeArm.goDown();
                break;
            }
            TrajectoryVelocityConstraint constraint2 = new TranslationalVelocityConstraint(5);
            TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                    .setVelConstraint(constraint2)
                    .strafeRight(x)
                    .build();
            drive.followTrajectorySequence(trajectorySequence);

            if (Math.abs(y) <= 0.1)
            {
                intakeArm.stopRetrieval();
            }
            else
            {
                intakeArm.moveRetrievalByInch(y);
            }
            drive.update();
        }
    }
}
