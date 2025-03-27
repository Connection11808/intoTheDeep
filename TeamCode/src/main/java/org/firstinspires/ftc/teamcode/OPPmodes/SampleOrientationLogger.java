package org.firstinspires.ftc.teamcode.OPPmodes;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sys.CornerDetectionProcessor;
import org.firstinspires.ftc.teamcode.sys.IntakeArm;
import org.firstinspires.ftc.teamcode.sys.RectDrawer;
import org.firstinspires.ftc.teamcode.sys.SampleLocalizer;
import org.firstinspires.ftc.teamcode.sys.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Sample Orientation Logger Calibrator", group = "Calibration")
public class SampleOrientationLogger extends LinearOpMode {
    private ElapsedTime grip_timer = new ElapsedTime();
    private final Size CAMERA_RESOLUTION = new Size(640, 480);

    public static double minusNinetyPosition = 0.03;
    public static double plusNinetyPosition = 0.36;

    private SampleOrientationProcessor processor;
    private CornerDetectionProcessor cornerDetectionProcessor;

    private Servo wrist;
    private WebcamName cameraName;
    private RectDrawer rectDrawer;
    private IntakeArm intakeArm;
    private Servo gripServo = null;
    private Servo sunServo = null;
    private Servo planetServo = null;
    private double grip_pos = 0;
    private double planet_pos = 0;
    private double gripClose = 0.01;
    private double planetUP = 0.359;
    private DcMotor retriveal = null;
    private Servo pitchServo = null;
    private double PitchPos = 0.765;
    SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException
    {
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        gripServo = hardwareMap.get(Servo.class, "grip_Servo");
        sunServo = hardwareMap.get(Servo.class, "sun_Servo");
        planetServo = hardwareMap.get(Servo.class, "planet_Servo");
        grip_pos = 0.38;
        gripServo.setDirection(Servo.Direction.REVERSE);
        planet_pos = planetUP;

        planetServo.setPosition(planet_pos);
        retriveal = hardwareMap.get(DcMotor.class, "retriveal");
        retriveal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retriveal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitchServo = hardwareMap.get(Servo.class, "pitch_Servo");
        pitchServo.setPosition(PitchPos);

        VisionPortal visionPortal = buildVisionPortal(cameraName);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        intakeArm = new IntakeArm(
                pitchServo,
                planetServo,
                sunServo,
                gripServo,
                retriveal);
        intakeArm.retSub();
        intakeArm.camera_goDown();
        waitForStart();
        while ((opModeInInit() || opModeIsActive()) && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            ;
        sleep(100);




        intakeArm.openGrip();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        intakeArm.camera_goDown();
        sleep(1000);
        moveRobotToSampleSUB(new Pose2d(0,0,Math.toRadians(0)), 0.0);


        while (opModeIsActive())
        {
            FtcDashboard.getInstance().getTelemetry().addData("position: ", retriveal.getCurrentPosition());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }

    public VisionPortal buildVisionPortal(WebcamName cameraName)
    {
        processor = new SampleOrientationProcessor(telemetry);
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

    private void updateExposure(VisionPortal visionPortal, long ms)
    {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(ms, TimeUnit.MILLISECONDS);  // exposure may have to be adjusted during competitions
    }

    private long getCorrectedExposure(double averageBrightness)
    {
        if (averageBrightness < 50) return 50;
        else if (averageBrightness < 80) return 27;
        else if (averageBrightness < 100) return 15;
        else if (averageBrightness < 140) return 14;
        else return 5;
    }
    public void moveRobotToSampleSUB(Pose2d startPose, double Xerror)
    {
        boolean goDownBool = false;
        double error;
        drive = new SampleMecanumDrive(hardwareMap);

        sleep(600);
        double y = rectDrawer.getReal_y();
        double x = rectDrawer.getRealX();

        TrajectoryVelocityConstraint constraint = new TranslationalVelocityConstraint(30);
        TrajectorySequence trajectorySequenceSTART = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x + 0.8)
                .build();
        TrajectorySequence trajectorySequenceSTART2 = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x + 0.8)
                .build();
        TrajectorySequence trajectorySequenceSTART23 = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(constraint)
                .strafeRight(x - 0.5)
                .build();

        ElapsedTime gripTimer = new ElapsedTime();
        if (Math.abs(rectDrawer.getSampleAngle()) < 45)
        {
            intakeArm.moveRetrievalByInch(y + 0.635 +(450 / 100.66));
            double servoAngle = 0.36;
            sunServo.setPosition(servoAngle);
            if (x < -1)
            {
                drive.followTrajectorySequence(trajectorySequenceSTART23);
            }
            else
            {
                drive.followTrajectorySequence(trajectorySequenceSTART2);
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
            intakeArm.moveRetrievalByInch(y + 0.8 + (450 / 100.66));
            drive.followTrajectorySequence(trajectorySequenceSTART2);
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

}