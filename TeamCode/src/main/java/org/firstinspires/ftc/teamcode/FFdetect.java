package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class FFdetect {
    private static final String TFOD_MODEL_ASSET = "teamelement.tflite";

    private static final String[] LABELS = {
            "cup"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AaUKnJ//////AAABmbcMAclE1kIAsN+jtie4MpYwfdVWRDQhkEvD6uQjqeiwwXNHR0KHwA7xLdqGAksc+r2arlo5Q2IhJohVlqZviAvryKs8pB8Q0ON7mB8lsNye53mPtRtmTlvpgGo1EFXjzlhcN9UUwBpod17qKcfSx48pIbG/DNU2KMDpwwnopqvfIqC3l/+FpkIHV/T0DM/2AeYywZyIV1RzIx5SGh8R9lVZr44IZnadQsNAHei/6Kd8b3nFVvE4F7vJX9kw9N3xXfOCBHzSTLW53aIeU9UL86Y04hjqxfry3oOv9PQ9K4D6AHTPa0JcOOq48pJ1o+WyOWu/meETom4tHmdVO1dCgiuSYD7UcDs2HfKlm0fh1zA1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private HardwareMap hardwareMap;

    public FFdetect(HardwareMap hardwareMap) { this.hardwareMap = hardwareMap; }

    public void init () {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(1.3, 2.3);
        }
    }

    /**
     * In the autonomous program, copy in the sample code to be able to use the method.
     * Example usage:
     *
     *   ObjectDetector ob = new ObjectDetector(hardwareMap);
     *   ob.init();
     *   String label = ob.detectRingLabel();
     *
     * @return the label of detected rings. possible values: "single", "quad", null (when nothing detected)
     */
    public String detectDuckPos() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            // step through the list of recognitions and display boundary info.
            int i = 0;
            if (updatedRecognitions.size() == 0) {
                return "right";
            }
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABELS[0])) {
                    if ((recognition.getLeft() + recognition.getRight()) / 2 < 320) {
                        return "left";
                    } else {
                        return "middle";
                    }
                }
            }
        }
        return "right";
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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