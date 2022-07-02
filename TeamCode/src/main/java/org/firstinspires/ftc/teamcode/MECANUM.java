package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name= "Mecanum", group = "LinearOpMode")
public class MECANUM extends LinearOpMode {
    private ElapsedTime CRuntime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotor Brat;
        DcMotor Intake;
        DcMotor Carusel;
        Servo servo;
        Servo TSE;
        ColorRangeSensor culoare;
        Servo TSEC;
        double y=0,a=0,b=0,lb=0, rb=0;
        double pozinit=0.8;
        double pozfin=0.3;
        double sus=0.75;
        double tse=0;
        double k=0;
        double pozinterm = 0.7;
        boolean PLIN=false;
        int BrJos=450;
        int BrMij=1000;
        int BrSus=1540;
        int up=1;
        int down=0;
        int rsb=0;
        double speed=1.5;

        Intake=hardwareMap.get(DcMotor.class, "Intake");
        Brat = hardwareMap.get(DcMotor.class, "Brat");
        servo = hardwareMap.get(Servo.class, "servo");
        Carusel = hardwareMap.get(DcMotor.class, "Carusel");
        culoare = hardwareMap.get(ColorRangeSensor.class, "culoare");
        TSE= hardwareMap.get(Servo.class, "TSE");
        TSEC= hardwareMap.get(Servo.class, "TSE-Cub");
        Brat.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
            Brat.setTargetPosition(0);
            Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
            Brat.setPower(0.6);
        servo.setPosition(pozinit);

        while(opModeIsActive())
        {
            TSEC.setPosition(tse);
            TSE.setPosition(sus);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            (double)(-gamepad1.left_stick_y/speed),
                            (double)(-gamepad1.right_stick_x/speed),
                            (double) (-gamepad1.left_stick_x/speed)
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.dpad_left)
                speed=2;
            if(gamepad1.dpad_up)
                speed=1.5;
            if(gamepad1.dpad_right)
                speed=1;

            if(gamepad2.left_bumper)//brat jos
            {
                Intake.setPower(0);
                servo.setPosition(pozinterm);
                Brat.setTargetPosition(BrJos);
                Intake.setPower(0);
                y++;
                sleep(200);

            }
            if (gamepad2.left_stick_button)
                Brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(gamepad2.right_bumper)//brat jos
            {
                Intake.setPower(0);
                servo.setPosition(pozinterm);
                Brat.setTargetPosition(BrMij);
                Intake.setPower(0);
                y++;
                sleep(200);

            }
            if(gamepad2.y)// brat sus
            {
                Intake.setPower(0);
                if(y%2==0)
                {
                    servo.setPosition(pozinterm);
                    Brat.setTargetPosition(BrSus);
                    Intake.setPower(0);
                }
                else
                {
                    down=0;
                    servo.setPosition(pozinit);
                    Brat.setTargetPosition(0);
                }
                y++;
                sleep(200);

            }
            if(gamepad2.b)
            {
                if(b%2==0)
                    Intake.setPower(0.8);
                else Intake.setPower(0);
                sleep(200);
                b++;
            }
            if(gamepad2.a)
            {
                if(a%2==0)
                {
                    servo.setPosition(pozinit);
                    Intake.setPower(-0.8);
                }
                else Intake.setPower(0);
                sleep(100);
                a++;
            }
            if(gamepad2.dpad_down)
            {
                if(down%2==0)
                servo.setPosition(pozfin);
                else if (down%2==1)
                servo.setPosition(pozinit);
                down++;
                sleep(100);
            }

            //carusel
            if(gamepad1.x)
                Carusel.setPower(0.7);
            else if(gamepad1.b)
                Carusel.setPower(-0.7);
            else Carusel.setPower(0);

            if(gamepad2.right_stick_button)
                rsb++;

            if(culoare.alpha()>600 && a%2==1 && rsb%2==0)
            {
                if(k==0) {
                    CRuntime.reset();
                    k++;
                }
                if(CRuntime.time()>0.3)
                    PLIN=true;
            }
            if(culoare.alpha()<200)
                k=0;
            if(PLIN)
            {
                Intake.setPower(0.5);
                servo.setPosition(pozinterm);
                PLIN= false;
                telemetry.addData("Cub?","da");
                a=0;
            }

            if(gamepad2.left_trigger!=0)
                sus += gamepad2.left_trigger/15.0;
            if(gamepad2.right_trigger!=0)
                sus-= gamepad2.right_trigger/15.0 ;
            if(gamepad2.dpad_up)
            {
                sleep(100);
                up++;
                if(up%2==0)
                    tse=0.5;
                if(up%2==1)
                    tse=0;
            }
            telemetry.addData("pozitie brat: ", Brat.getCurrentPosition());
            telemetry.addData("Cub?", "NU");
            telemetry.addData("TSE: ", TSE.getPosition());
            telemetry.update();
        }
    }
}