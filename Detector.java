package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

public class Detector {
    
    /**
     * path to model (upload in the OnBotJava Manage tab)
    */
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/reversed.tflite";
    
    /**
     * array with all possible labels
    */
    private static final String[] LABELS = {
      "1",
      "2",
      "3"
    };
    
    /**
     * generate Vuforia key and place it here
    */
    private static final String VUFORIA_KEY =
            "Ae8ACLv/////AAABmapP7OaP00Peg5RTrdBlFlAveb2gBnRlqKGEO4M7pFytVooHWvKZoOdXTSbHDTKRtur/QzaHkNgNq55RtFBPO94R0HvwRlEq7uTZj9nX5hy6YLnFsjgyaGrLn3Ddv5cf5NolmGeW4Em5Ix0kUrUOQPSwImYGKK6NdGmQasKoKOcDVmuV5V+uUe5ByLamNwj/+SUqV8vXJnDlDnns4rt5a+55I2fq8sr7rdbIq9tXguNJABvPCV8FdNmTrk9SZgNV7RPoiCHM5DGJvxjwd/R2wazmz6gtJ4EGR4w4ffZBziSjRL/4DUqS7/byM6Dq1lyj7l09NcMjqcUqWskF06VgVw8CWRgVkMSjv5RbW6Oclebm";

    
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private List<Recognition> updatedRecognitions; 
    private ArrayList<String> Recognitions = new ArrayList<String>();
    private ArrayList<Double> Certainty = new ArrayList<Double>();
    public HardwareMap hwMap;
    
    
    /**
     * constructor
     * initialize object detection model
     * @param hwMap hardware map from the phone
    */
    public Detector(HardwareMap hwMap){
        this.hwMap = hwMap;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }
    
    /**
     * for each recognition, draw a box around the object in the frame
    */
    public void drawBox(){
        if (tfod != null) {
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                    Recognitions.add(recognition.getLabel());
                    Certainty.add((double)(recognition.getConfidence()));
                }
            }
        }
    }
    
    //recognition.getLabel(), recognition.getConfidence() * 100 
    
    /**
     * initialize the vuforia augmented reality engine
    */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * method to setup tensorflow
     * loads the model
     * loads label map
     * sets input and ouput settings
    */ 
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
    /**
     * method to get the strongest prediction that the model makes
     * finds the prediction in Recognitions with the highest confidence level and uses that
     * @return the desired parking space (1-3), with 9 occuring if there are no predictions
    */
    public int getPrediction(){
        double max = 0;
        int maxIndex = 0;
        
        for (int index = 0; index < Certainty.size(); index++){
            if(Certainty.get(index) > max){
                max = Certainty.get(index);
                maxIndex = index;
            }
        }
        
        if(Recognitions.size() > 0){
            return(Integer.parseInt(Recognitions.get(maxIndex)));
        }
        else {
            return(9);
        }
    }
}