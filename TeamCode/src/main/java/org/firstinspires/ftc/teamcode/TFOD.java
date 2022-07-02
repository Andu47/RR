
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@TeleOp(name = "Concept: TensorFlow Object Detection", group = "LinearOpMode")

public class TFOD extends LinearOpMode {
    double pozitie=0;
    private static final String TFOD_MODEL_ASSET = "SoareCOMPLET.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    private static final String VUFORIA_KEY =
            "AQXzaeH/////AAABma+dmoRw1kZHgWbfDr88vn4I2y/JjiEnuuQvCZjhNbwWZE1CdaCGcKPWc5Pot143CxXBXDQyqZMQTyDqbBXzxe5YovSPVlPpa0LiIecLfVPYVnWkqng1tm8B1RNaeyYx25fkGI/LAu3Qeq/KlVPHN+iUClyDowkjXejs8+wzBp6wdzUEjzwEcnrkruqirmZEDtwnPKKHHOItKj9n7TzS77RtS6fSF0P9Qtlfi58Lg3kT3v/6ml7slcQMCGIWbBZRA9EYatWvK6ffM/TL9Xvr0jqaAiho0fQiilxGg2GvMRzc3qFyqA4TwPLrvn41C620ufoVvPAgmk7+at56tpE7/Y9hugGE1ML8jZRdqx9iqC6r";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("pozitie: ", (recognition.getTop()+recognition.getBottom())/2.0);
                            pozitie=(recognition.getTop()+recognition.getBottom())/2.0;
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.addData("Pozitie: ", pozitie);
                        telemetry.update();
                    }
                }
            }
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