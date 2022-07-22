
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name="TeleOp")

public class TeleOP extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor st;
    private DcMotor dr;
    private Servo directie;
    private DistanceSensor dist_dr;
    private DistanceSensor dist_st;
    private OpenCvCamera webcam;
    int s = 0, val, spate = 0, laterala = 0;
    double mers;
    double power;
    boolean press=false, press2=false;
    long last_pressed_front=0,last_pressed_ps=0;
    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    public static Scalar scalarLowerYCrCbg = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCbg = new Scalar(255.0, 170.0, 120.0);

    public static Scalar scalarLowerYCrCbv = new Scalar(  0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCbv = new Scalar(255.0, 120.0, 120.0);
    @Override
    public void runOpMode() {
        st  = hardwareMap.get(DcMotor.class, "st");
        dr = hardwareMap.get(DcMotor.class, "dr");
        directie = hardwareMap.get(Servo.class, "directie");
        dist_dr= hardwareMap.get(DistanceSensor.class, "dist_dr");
        dist_st = hardwareMap.get(DistanceSensor.class, "dist_st");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.cross && s==0)
            {
                s++;
                Controller();
            }
            if(gamepad1.circle && s==0)
            {
                s++;
                Parcare();
                st.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                st.setPower(0);
                dr.setPower(0);
            }
            if(gamepad1.triangle && s==0)
            {
                s++;
                Line_Follow();
                webcam.closeCameraDevice();
            }
            if(gamepad1.dpad_up && s==0)
            {
                s++;
                AI_Rosu();
                webcam.closeCameraDevice();
            }
            if(gamepad1.dpad_left && s==0)
            {
                s++;
                AI_Galben();
                webcam.closeCameraDevice();
            }
            if(gamepad1.dpad_right && s==0)
            {
                s++;
                AI_Verde();
                webcam.closeCameraDevice();
            }
            telemetry.addLine("Start");
            telemetry.update();
            NIMIC();
        }
    }
    public void Parcare() {
        while (s == 1 && opModeIsActive()) {
            if (s == 1) {
                st.setPower(-0.15);
                dr.setPower(0.15);
                directie.setPosition(0.69);
                val = (int) dist_st.getDistance(DistanceUnit.CM);
                telemetry.addData("Stanga", dist_st.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            val = (int) dist_st.getDistance(DistanceUnit.CM);
            telemetry.addData("Stanga", dist_st.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (val > 50 && spate == 0 && laterala == 0) {
                st.setPower(0);
                dr.setPower(0);
                st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                st.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                dr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spate++;

            }
            val = (int) dist_st.getDistance(DistanceUnit.CM);
            telemetry.addData("Stanga", dist_st.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (val > 40 && val < 50 && laterala == 0 && spate == 0) {
                st.setPower(0);
                dr.setPower(0);
                st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                st.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                dr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                laterala++;
            }
            if (val < 40 && laterala != 0) {
                st.setPower(0);
                dr.setPower(0);
                mers = dr.getCurrentPosition() * 3.1415 * 10 / 537.7;
                telemetry.addLine("Laterala");
                telemetry.addData("Dist", mers);
                telemetry.update();
                lateral();
                st.setPower(0);
                dr.setPower(0);

            } else if (val < 40 && spate != 0) {
                st.setPower(0);
                dr.setPower(0);
                mers = dr.getCurrentPosition() * 3.1415 * 10 / 537.7;
                telemetry.addLine("Spate");
                telemetry.addData("Dist", mers);
                telemetry.update();
                spate();
                st.setPower(0);
                dr.setPower(0);
            }
            val = (int) dist_st.getDistance(DistanceUnit.CM);
            telemetry.addData("Stanga", dist_st.getDistance(DistanceUnit.CM));
            telemetry.addData("Spate", spate);
            telemetry.addData("Lateral", laterala);
            telemetry.update();
            if(gamepad1.share)
            {
                s=0;
            }
        }
    }
    public void lateral () {
        double lateral1 = 45 * 537.7 / 10 / 3.1415;
        double fata = 20 * 537.7 / 10 / 3.1415;
        telemetry.addData("Parcare", "laterala");
        telemetry.update();
        st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.setTargetPosition((int) -fata);
        dr.setTargetPosition((int) fata);
        st.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        st.setPower(-0.15);
        dr.setPower(0.15);
        telemetry.addData("Mers", "fata");
        telemetry.update();
        sleep(2000);
        st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.setTargetPosition((int) lateral1);
        st.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        directie.setPosition(0.53);
        st.setPower(0.15);
        telemetry.addData("Mers", "stanga");
        telemetry.update();
        sleep(2000);
        dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr.setTargetPosition((int) -lateral1);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        directie.setPosition(0.86);
        dr.setTargetPosition((int) -lateral1);
        dr.setPower(-0.15);
        telemetry.addData("Mers", "dreapta");
        telemetry.update();
        sleep(2000);
        directie.setPosition(0.7);
        s=0;
        laterala = 0;
    }
    public void spate () {
        double spate1 = 45 * 537.7 / 10 / 3.1415;
        double spate2 = 30 * 537.7 / 10 / 3.1415;
        double fata = 20 * 537.7 / 10 / 3.1415;
        telemetry.addData("Parcare", "spate");
        telemetry.update();
        st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.setTargetPosition((int) -fata);
        dr.setTargetPosition((int) fata);
        st.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        st.setPower(-0.15);
        dr.setPower(0.15);
        telemetry.addData("Mers", "fata");
        telemetry.update();
        sleep(2000);
        st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        directie.setPosition(0.53);
        st.setTargetPosition((int) spate1);
        st.setPower(0.15);
        telemetry.addData("Mers", "stanga");
        telemetry.update();
        sleep(2800);
        st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        st.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        directie.setPosition(0.7);
        st.setTargetPosition((int) spate2);
        dr.setTargetPosition((int) -spate2);
        st.setPower(0.15);
        dr.setPower(-0.15);
        telemetry.addData("Mers", "spate");
        telemetry.update();
        sleep(1500);
        st.setPower(0);
        dr.setPower(0);
        spate = 0;
        s=0;
    }
    public void Controller() {
        while (s == 1 && opModeIsActive()) {
            if (gamepad1.left_trigger == 1) {
                if (power > -0.1)
                    power = power - 0.2;
                st.setPower(-power);
                dr.setPower(power);
                directie.setPosition(-0.2 * gamepad1.left_stick_x + 0.7);
            }
            directie.setPosition(-0.2 * gamepad1.left_stick_x + 0.7);
            if (gamepad1.x && System.currentTimeMillis() - last_pressed_front > 500) {
                last_pressed_front = System.currentTimeMillis();
                press = !press;
            }
            if(gamepad1.ps && System.currentTimeMillis() - last_pressed_ps > 500)
            {
                last_pressed_ps =  System.currentTimeMillis();
                press2 = !press2;
            }
            while (dist_dr.getDistance(DistanceUnit.CM) < 30 && press) {
                st.setPower(0.1);
                dr.setPower(-0.1);
            }
            //ACC
            if (press2) {
                power= gamepad1.right_trigger*(1.6*dist_dr.getDistance(DistanceUnit.CM)-16)/100;
                st.setPower(-power);
                dr.setPower(power);
            }
            if(press2==false)
            {
                power = gamepad1.right_trigger;
                st.setPower(-power);
                dr.setPower(power);
            }
            if (gamepad1.atRest())
                directie.setPosition(-0.2 * gamepad1.left_stick_x + 0.7);
            telemetry.addData("Front assist", press);
            telemetry.addData("ACC", press2);
            telemetry.addData("Fata", dist_dr.getDistance(DistanceUnit.CM));
            telemetry.update();
            if(gamepad1.share)
            {
                s=0;
            }
        }
    }
    public void Line_Follow() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        LineDetection myPipeline;
        webcam.setPipeline(myPipeline = new LineDetection(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        st  = hardwareMap.get(DcMotor.class, "st");
        dr = hardwareMap.get(DcMotor.class, "dr");
        directie = hardwareMap.get(Servo.class, "directie");
        telemetry.update();
        while (s==1 && opModeIsActive())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            if (myPipeline.getRectArea() > 2000) {
                directie.setPosition(myPipeline.getRectMidpointX()*-0.00066+0.9);
                st.setPower(-0.2);
                dr.setPower(0.2);


            }
            else
            {
                NIMIC();
            }
            if(gamepad1.share)
            {
                s=0;
            }
            telemetry.addLine("LineFollow");
            telemetry.update();
        }
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void NIMIC(){
        st.setPower(0);
        dr.setPower(0);
        directie.setPosition(0.7);
    }
    public void AI_Rosu(){
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        st  = hardwareMap.get(DcMotor.class, "st");
        dr = hardwareMap.get(DcMotor.class, "dr");
        directie = hardwareMap.get(Servo.class, "directie");
        telemetry.update();
        while (s==1 && opModeIsActive())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            if (myPipeline.getRectArea() < 20000 && myPipeline.getRectArea() > 2000) {
                directie.setPosition(myPipeline.getRectMidpointX()*-0.00066+0.9);
                st.setPower(-(-0.0000078*myPipeline.getRectArea()+0.315));
                dr.setPower(-0.0000078*myPipeline.getRectArea()+0.315);


            }
            else
            {
                NIMIC();
            }
            if(gamepad1.share)
            {
                s=0;
            }
            telemetry.addLine("AI_ROSU");
            telemetry.update();
        }
    }
    public void AI_Galben(){
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            //OpenCV Pipeline
            ContourPipelineg myPipeline;
            webcam.setPipeline(myPipeline = new ContourPipelineg(borderLeftX,borderRightX,borderTopY,borderBottomY));
            // Configuration of Pipeline
            myPipeline.configureScalarLower(scalarLowerYCrCbg.val[0],scalarLowerYCrCbg.val[1],scalarLowerYCrCbg.val[2]);
            myPipeline.configureScalarUpper(scalarUpperYCrCbg.val[0],scalarUpperYCrCbg.val[1],scalarUpperYCrCbg.val[2]);
            // Webcam Streaming
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
            // Only if you are using ftcdashboard
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            FtcDashboard.getInstance().startCameraStream(webcam, 10);
            st  = hardwareMap.get(DcMotor.class, "st");
            dr = hardwareMap.get(DcMotor.class, "dr");
            directie = hardwareMap.get(Servo.class, "directie");
            telemetry.update();
            waitForStart();
            while (s==1 && opModeIsActive())
            {
                myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
                if(myPipeline.error){
                    telemetry.addData("Exception: ", myPipeline.debug);
                }

                if (myPipeline.getRectArea() < 20000 && myPipeline.getRectArea() > 2000) {
                    directie.setPosition(myPipeline.getRectMidpointX()*-0.00066+0.9);
                    st.setPower(-(-0.0000078*myPipeline.getRectArea()+0.315));
                    dr.setPower(-0.0000078*myPipeline.getRectArea()+0.315);


                }
                else
                {
                    NIMIC();
                }
                if(gamepad1.share)
                {
                    s=0;
                }
                telemetry.addLine("AI_GALBEN");
                telemetry.update();
            }
    }
    public void AI_Verde(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipelinev myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipelinev(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCbv.val[0],scalarLowerYCrCbv.val[1],scalarLowerYCrCbv.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCbv.val[0],scalarUpperYCrCbv.val[1],scalarUpperYCrCbv.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        st  = hardwareMap.get(DcMotor.class, "st");
        dr = hardwareMap.get(DcMotor.class, "dr");
        directie = hardwareMap.get(Servo.class, "directie");
        telemetry.update();
        while (s==1 && opModeIsActive())
        {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            if (myPipeline.getRectArea() < 20000 && myPipeline.getRectArea() > 2000)
            {
                directie.setPosition(myPipeline.getRectMidpointX()*-0.00066+0.9);
                st.setPower(-(-0.0000078*myPipeline.getRectArea()+0.315));
                dr.setPower(-0.0000078*myPipeline.getRectArea()+0.315);
            }
            else
            {
                NIMIC();
            }
            if(gamepad1.share)
            {
                s=0;
            }
            telemetry.addLine("AI_VERDE");
            telemetry.update();
        }
    }
}
