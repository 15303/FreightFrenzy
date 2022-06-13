package org.firstinspires.ftc.teamcode.baloney;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import minipid.MiniPID;

import static com.sun.tools.javac.jvm.ByteCodes.error;

public class Baloney {

    public int color = BLUE;
    public static int BLUE = 1;
    public static int RED = -1;

    public DcMotor left, right;
    public DcMotor lifter;
    public CRServo grabber;
    public DcMotor spinner;
    public BNO055IMU imu;

    private MiniPID pid, rpid;
    private static final String VUFORIA_KEY =
            "AV1AlC//////AAABmZ8844uwbEHrg1LUsHVFKxlStW4C7oPMwyIXaVB2lFgVrXI7AcN37g06/oHM+7Smo0UtpZXtGANu2IWFTeqOdHO83zy8s3nw7ZfZ60OUz9L230sZ0liJbP8aeIKa0a0ibeL+mH4zJTOHU/3rdfcv8PbufYdeMh1ImaoFXTXQkMqiELuxK32/kvH/sRyvMg5JmoQDxKgSgNhN/Vle754F6hCOVk1alZE7H5gXHifhPtL0Gf+AhkrfsbKi+zeZ3gRoGLzX54Qq8EUmOhlm5+ZMbdYkx1F4u8FoLczDK+Qt4J23kEqkbCA5HyDJJsmyA30/fEIYDEepO9f86U96LfOCIFt8Q3vFCWSq4IJZphMlVOF7";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry out;

    private LinearOpMode opMode;

    public Baloney(LinearOpMode opMode) {
        this.opMode = opMode;
        this.out = opMode.telemetry;

        initMotors();
        initServos();
        initIMU();
        initPID();
//        initVuforia();
//        initTfod();
    }

    private void initMotors() {
        left = opMode.hardwareMap.get(DcMotor.class, "left");
        right = opMode.hardwareMap.get(DcMotor.class, "right");

        lifter = opMode.hardwareMap.get(DcMotor.class, "lifter");
        spinner = opMode.hardwareMap.get(DcMotor.class, "spinner");

        left.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initServos() {
        grabber = opMode.hardwareMap.get(CRServo.class, "grabber");
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initPID(){
        // rpid: .01, 3.804e-6, -1.913
        initPID(1,0,0,0.01,0,0);
    }

    public void initPID(double p, double i, double d, double rp, double ri, double rd) {
        pid = new MiniPID(p, i, d);
        pid.setOutputLimits(-1.0, 1.0);
        pid.setOutputFilter(0.03); //1629 to 1927
        rpid = new MiniPID(rp, ri, rd); //0.80 0.55 -4.4

    }

    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("FreightFrenzy_BCDM.tflite", "Ball", "Cube", "Duck", "Marker");
        tfod.activate();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        left.setMode(runMode);
        right.setMode(runMode);
    }

    public void setPower(double l, double r) {
        left.setPower(l);
        right.setPower(r);
    }

    public void setColoredPower(double l, double r) {
        if(color == BLUE) {
            left.setPower(l);
            right.setPower(r);
        } else {
            left.setPower(r);
            right.setPower(l);
        }
    }

    public void setPower(double power) {
        setPower(power, power);
    }

    public void setTurningPower(double power) {
        setPower(color*power, -color*power);
    }

    public int currentPosition() {
        return (left.getCurrentPosition() + right.getCurrentPosition())/2;
    }
    private long now() {
        return System.currentTimeMillis();
    }
    private boolean active() {
        return opMode.opModeIsActive();
    }
    private double currentHeading() {
        return -imu.getAngularOrientation().firstAngle;
    }
    private double headingError(double goal) {
        return normalizeDegrees(currentHeading() - goal);
    }

    private double normalizeDegrees(double degrees){
        degrees %= 360;
        if(degrees > 180) degrees -= 360;
        if(degrees <= -180) degrees += 360;
        return degrees;
    }

    public void turn(double degrees) {
        rpid.reset();

        double goal = normalizeDegrees(currentHeading() + degrees);

        long until = now() + 2500; //expiration
        while(now() < until && Math.abs(headingError(goal)) > 5 && active()) {
            double output = rpid.getOutput(headingError(goal), 0);
            setTurningPower(output);
            opMode.sleep(20);

            opMode.telemetry.addData("now, until", now() + ", " + until);
            opMode.telemetry.addData("headingError", headingError(goal));
            opMode.telemetry.addData("rpid output", output);
            opMode.telemetry.update(); // rpid:
        }
        until = now() + 700;
        while (now() < until && active()) {
            double output = rpid.getOutput(headingError(goal), 0);
            setTurningPower(output);
            opMode.sleep(20);
        }

        setPower(0.0);
    }

    public void travelFor(long ms, double power) {
        setPower(power);
        opMode.sleep(ms);
        setPower(0.0);
    }

    public void travelFor(long ms) {
        travelFor(ms, 0.75);
    }


    public void travelPID(double inches) {
        double accel = 0.3;
        int steps = (int)(inches * 31);
        int goal = currentPosition() + steps;
        pid.reset();

        long until = (long)(now() + inches * 500 + 500);
        while(now() < until && Math.abs(currentPosition() - goal) > 20 && active()) {
            double output = pid.getOutput(currentPosition(), goal);
            accel += 0.05;
            setPower(Math.min(output, accel));
            opMode.sleep(50);

            out.addData("diff", currentPosition() - goal);
            out.addData("output", output);
            out.update();
        }
        until = now() + 200;
        while(now() < until && active()) {
            double output = pid.getOutput(currentPosition(), goal);
            setPower(output);
            opMode.sleep(50);
        }
        setPower(0);
    }


    public void setSpinPower(double power) {
        spinner.setPower(color*power);
    }

    public void spinFor(long ms, double power) {
        spinner.setPower(color*power);
        opMode.sleep(ms);
        spinner.setPower(0.0);
    }

    public void spinFor(long ms) {
        spinFor(ms, 0.3);
    }

    public void liftFor(long ms, int direction) {
        lifter.setPower(direction * 0.7);
        opMode.sleep(50);
        lifter.setPower(0.0);
    }

    public void hand(int direction) {
        grabber.setPower(direction * 1.0);
        opMode.sleep(200);
        grabber.setPower(0);
    }

    public int recognize() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        for (Recognition recognition : updatedRecognitions) {
            if(recognition.getLabel().equals("Duck") || recognition.getLabel().equals("Marker")) {
                float x = (recognition.getLeft() + recognition.getRight())/2;
                if(x < 0.33) {
                    return 0;
                } else if(x < 0.67) {
                    return 1;
                } else {
                    return 2;
                }
            }
        }
        return -1;
    }

    public void team(int color) {
        this.color = color;
    }

    public void turnFor(long ms, double power) {
        setTurningPower(power);
        opMode.sleep(ms);
        setPower(0.0);
    }
}
