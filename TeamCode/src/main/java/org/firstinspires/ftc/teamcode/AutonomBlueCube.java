package org.firstinspires.ftc.teamcode;
import android.content.pm.ResolveInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;
import java.util.List;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.TimeUnit;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

@Autonomous (name= "BlueCube")

public class AutonomBlueCube extends LinearOpMode {
    DcMotor Brat;
    DcMotor Intake;
    DcMotor FataSt;
    DcMotor SpateSt;
    DcMotor FataDr;
    DcMotor SpateDr;
    DcMotor Carusel;
    Servo servo;
    Servo TSE;
    Servo TSEC;
    ColorRangeSensor culoare;
    ColorRangeSensor culPos;
    DistanceSensor DS;
    DistanceSensor DD;
    private static final String TFOD_MODEL_ASSET = "SoareCOMPLET.tflite";
    private static final String[] LABELS = {
            "TSE"
    };
    private ElapsedTime CRuntime = new ElapsedTime();
    private ElapsedTime durataMeci = new ElapsedTime();
    private static final String VUFORIA_KEY =
            "AQXzaeH/////AAABma+dmoRw1kZHgWbfDr88vn4I2y/JjiEnuuQvCZjhNbwWZE1CdaCGcKPWc5Pot143CxXBXDQyqZMQTyDqbBXzxe5YovSPVlPpa0LiIecLfVPYVnWkqng1tm8B1RNaeyYx25fkGI/LAu3Qeq/KlVPHN+iUClyDowkjXejs8+wzBp6wdzUEjzwEcnrkruqirmZEDtwnPKKHHOItKj9n7TzS77RtS6fSF0P9Qtlfi58Lg3kT3v/6ml7slcQMCGIWbBZRA9EYatWvK6ffM/TL9Xvr0jqaAiho0fQiilxGg2GvMRzc3qFyqA4TwPLrvn41C620ufoVvPAgmk7+at56tpE7/Y9hugGE1ML8jZRdqx9iqC6r";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double y=0,a=0,b=0,x=3;
    double pozitie=0;
    double pozinit=0.8;
    double pozfin=0.3;
    double sus=0.8;
    double tse=0;
    double k=0;
    int k1=0;
    double pozinterm = 0.7;
    boolean PLIN=false;
    int BrSus=1540;
    int up=0;
    int down=0;
    double speed=1.5;
    @Override
    public void runOpMode() {
        OpMode robot = null;
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Brat = hardwareMap.get(DcMotor.class, "Brat");
        FataSt = hardwareMap.get(DcMotor.class, "FataSt");
        SpateSt = hardwareMap.get(DcMotor.class, "SpateSt");
        FataDr = hardwareMap.get(DcMotor.class, "FataDr");
        SpateDr = hardwareMap.get(DcMotor.class, "SpateDr");
        servo = hardwareMap.get(Servo.class, "servo");
        TSEC = hardwareMap.get(Servo.class, "TSE-Cub");
        TSE = hardwareMap.get(Servo.class, "TSE");
        culoare = hardwareMap.get(ColorRangeSensor.class, "culoare");
        culPos = hardwareMap.get(ColorRangeSensor.class, "culoare spate");
        DD= hardwareMap.get(DistanceSensor.class, "distStanga");
        DS = hardwareMap.get(DistanceSensor.class, "distDreapta");
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);
        FataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        SpateSt.setDirection(DcMotorSimple.Direction.REVERSE);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
        TSE.setPosition(sus);
        waitForStart();

        durataMeci.reset();

        if (opModeIsActive()) {
            Brat.setTargetPosition(0);
            Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
            Brat.setPower(0.6);
            sleep(200);
            servo.setPosition(pozinterm);
            TSEC.setPosition(tse);
            TSE.setPosition(sus);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            if (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions)
                            pozitie=(recognition.getTop()+recognition.getBottom())/2.0;
                        telemetry.update();
                    }
                }
            }

            Pose2d startPose = new Pose2d(0, 0, 0);
            Pose2d startPoseB = new Pose2d(0,0,3.14);
            Pose2d shippingPose = new Pose2d(14,25,4.13);
            Pose2d colorSensPose = new Pose2d(-24.5,2.4,3.14);

            drive.setPoseEstimate(startPose);

            Trajectory fromStartToShip1 = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(shippingPose)
                    .build();
            Trajectory fromStartToShip = drive.trajectoryBuilder(startPoseB)
                    .lineToLinearHeading(shippingPose)
                    .build();
            Trajectory fromShipToStart = drive.trajectoryBuilder(shippingPose)
                    .lineToLinearHeading(startPoseB)
                    .build();
            Trajectory fromColorToStart = drive.trajectoryBuilder(colorSensPose)
                    .lineToLinearHeading(startPoseB)
                    .build();
            Trajectory dreapta = drive.trajectoryBuilder(startPose)
                    .strafeRight(25)
                    .build();
            Trajectory stanga = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .strafeLeft(1 )
                    .build();
            Trajectory fata = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .forward(5)
                    .build();
            Trajectory fataM = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .forward(35)
                    .build();
            Trajectory spate = drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .back(5)
                    .build();

            if(pozitie<600 && pozitie>10)//sus
            {
                telemetry.addData("Pozitie:", pozitie);
                x=2;
                Brat.setTargetPosition(1550);
            }

            else if(pozitie>600)//mijloc
            {
                telemetry.addData("Pozitie:", pozitie);
                x=1;
                Brat.setTargetPosition(1000);
            }

            else//jos
            {
                telemetry.addData("Pozitie:", pozitie);
                x=3;
                Brat.setTargetPosition(400);
            }

            telemetry.update();

            drive.followTrajectory(fromStartToShip1);

            servo.setPosition(pozfin);
            sleep(450);
            servo.setPosition(pozinit);

            Brat.setTargetPosition(0);
            drive.followTrajectory(fromShipToStart);
            ////////////////////////////////////////////////////////////////////
            for(int i=0; i<=1; i++)
            {
                Intake.setPower(-1);
                drive.setPoseEstimate(new Pose2d(0,0,0));
                drive.followTrajectory(stanga);

                Pose2d thisPos= drive.getPoseEstimate();
                Trajectory fataCub= drive.trajectoryBuilder(thisPos)
                        .forward(30+(k1+5)*i)
                        .build();
                drive.followTrajectory(fataCub);
                k=0;
                PLIN=false;
                ElapsedTime timp1 = new ElapsedTime();
                while(!PLIN)
                {
                    if (timp1.time(TimeUnit.MILLISECONDS)>1500 && timp1.time(TimeUnit.MILLISECONDS)<2500)
                    {
                        Intake.setPower(0.5);
                        drive.setMotorPowers(-0.25,-0.3,-0.25,-0.3);
                    }

                    if (timp1.time(TimeUnit.MILLISECONDS)<1500 ||  timp1.time(TimeUnit.MILLISECONDS)>2500)
                    {
                        Intake.setPower(-1);
                        drive.setMotorPowers(0.2,0.2,0.2,0.2);
                    }
                    if(culoare.alpha()>600)
                    {
                        if(k==0) {
                            CRuntime.reset();
                            timp1.reset();
                            k++;
                        }
                        if(CRuntime.time()>0.3)
                        {
                            PLIN=true;
                            k=0;
                        }
                    }
                    if (culoare.alpha()<200)
                        k=0;
                    if(timp1.time(TimeUnit.SECONDS)>6)
                        FataDr.setTargetPosition(FataDr.getCurrentPosition()+20);
                    FataSt.setTargetPosition(FataSt.getCurrentPosition()+20);
                    SpateDr.setTargetPosition(SpateDr.getCurrentPosition()+20);
                    SpateSt.setTargetPosition(SpateSt.getCurrentPosition()+20);
                    drive.setMotorPowers(0.2,0.2,0.2,0.2);
                }

                drive.setMotorPowers(0,0,0,0);

                telemetry.addData("heading: ", drive.getPoseEstimate().getHeading());
                telemetry.update();

                Intake.setPower(0.5);
                servo.setPosition(pozinterm);

                while(culPos.alpha()<250)
                {
                    FataDr.setTargetPosition(FataDr.getCurrentPosition()-100);
                    FataSt.setTargetPosition(FataSt.getCurrentPosition()-70);
                    SpateDr.setTargetPosition(SpateDr.getCurrentPosition()-70);
                    SpateSt.setTargetPosition(SpateSt.getCurrentPosition()-100);
                    drive.setMotorPowers(-0.3,-0.2,-0.3,-0.2);
                }
                drive.setMotorPowers(0,0,0,0);
                drive.setPoseEstimate(new Pose2d(0,0,0));
                drive.followTrajectory(stanga);
                drive.setPoseEstimate(colorSensPose);

                drive.followTrajectory(fromColorToStart);

                Brat.setTargetPosition(BrSus);
                drive.followTrajectory(fromStartToShip);
                servo.setPosition(pozfin);
                sleep(450);
                servo.setPosition(pozinit);
                Brat.setTargetPosition(0);
                drive.followTrajectory(fromShipToStart);
            }
            drive.setPoseEstimate(new Pose2d(0,0,0));
            drive.followTrajectory(fataM);
            drive.setPoseEstimate(new Pose2d(0,0,0));
            drive.followTrajectory(dreapta);
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
