package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name= "Teste" , group = "LinearOpMode")
public class ColorV3 extends LinearOpMode {
    private ElapsedTime CRuntime = new ElapsedTime();
    public DcMotor Brat;
    public DcMotor FataDr;
    public DcMotor FataSt;
    public DcMotor SpateDr;
    public DcMotor SpateSt;
    public DcMotor Intake;
    public DcMotor Carusel;
    public Servo servo;
    public ColorRangeSensor culoare;
    public ColorRangeSensor culoarePositie;
    public DistanceSensor DS;
    public DistanceSensor DD;
    public double y = 0, a = 0, b = 0;
    public int k=0;
    public double t;
    public double pozinit = 0.81;
    public double pozfin = 0.3;
    public double pozinterm = 0.6;
    public double sus = 0;
    boolean PLIN=false;
    @Override
    public void runOpMode() throws InterruptedException {
        Intake  = hardwareMap.get(DcMotor.class, "Intake");
        Brat    = hardwareMap.get(DcMotor.class, "Brat");
        servo   = hardwareMap.get(Servo.class, "servo");
        Carusel = hardwareMap.get(DcMotor.class, "Carusel");
        FataDr  = hardwareMap.get(DcMotor.class, "FataDr");
        FataSt  = hardwareMap.get(DcMotor.class, "FataSt");
        SpateDr = hardwareMap.get(DcMotor.class, "SpateDr");
        SpateSt = hardwareMap.get(DcMotor.class, "SpateSt");
        culoare = hardwareMap.get(ColorRangeSensor.class, "culoare");
        culoarePositie = hardwareMap.get(ColorRangeSensor.class, "culoare spate");
        DD = hardwareMap.get(DistanceSensor.class, "distDreapta");
        DS = hardwareMap.get(DistanceSensor.class, "distStanga");

        FataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        SpateSt.setDirection(DcMotorSimple.Direction.REVERSE);
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        servo.setPosition(pozinit);
        CRuntime.reset();
        while(opModeIsActive())
        {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Brat: ", Brat.getCurrentPosition());
            telemetry.addData("Distanta: ", culoare.getDistance(DistanceUnit.CM));
            telemetry.addData("culoare ", culoare.alpha());
            telemetry.addData("Distanta Stanga: ", DS.getDistance(DistanceUnit.CM));
            telemetry.addData("Distanta Dreapta: ", DD.getDistance(DistanceUnit.CM));
            telemetry.addData("culoare jos:", culoarePositie.alpha());
            telemetry.addData("Status", "Run Time: " + CRuntime.toString());
            AngularVelocity ang = drive.imu.getAngularVelocity();

            telemetry.addData("Current IMU Ang Velo Deg X", Math.toDegrees(ang.xRotationRate));
            telemetry.addData("Current IMU Ang Velo Deg Y", Math.toDegrees(ang.yRotationRate));
            telemetry.addData("Current IMU Ang Velo Deg Z", Math.toDegrees(ang.zRotationRate));
            telemetry.addData("Current IMU Ang Velo Unit", ang.unit);
            telemetry.update();
            if(gamepad1.right_stick_button)
                Brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(gamepad1.x)
                servo.setPosition(pozinterm);
            if(gamepad1.y)
                servo.setPosition(pozinit);
            if(culoare.alpha()>2000)
            {
                if(k==0) {
                    CRuntime.reset();
                    k++;
                }
                if(CRuntime.time()>2)
                    PLIN=true;
            }
            if(culoare.alpha()<400)
                k=0;
            if(PLIN)
            {
                servo.setPosition(pozinterm);
                Intake.setPower(-0.5);
            }
            Trajectory dreapta =drive.trajectoryBuilder(new Pose2d(0,0,0))
                    .back(5)
                    .build();
            if(gamepad1.dpad_up)
                drive.followTrajectory(dreapta);
        }
    }
}
