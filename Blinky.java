package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

class Blinky {
    DcMotor one = null;
    DcMotor two = null;
    DcMotor three = null;
    DcMotor four = null;

    //CRServo leftintake = null;
    //CRServo rightintake = null;
    //DcMotor track = null;
    DcMotor sidelift = null;
    //DcMotor frontlift = null;
    //CRServo frontliftgrab = null;
    Servo sideliftgrab = null;
    Servo platform = null;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    HardwareMap hwMap = null;

    ColorSensor color;
    DistanceSensor distance;

    Blinky(){}

    void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        one = ahwMap.dcMotor.get("one");
        two = ahwMap.dcMotor.get("two");
        three = ahwMap.dcMotor.get("three");
        four = ahwMap.dcMotor.get("four");

        //leftintake = ahwMap.crservo.get("leftintake");
        //rightintake = ahwMap.crservo.get("rightintake");
        //track = ahwMap.dcMotor.get("track");
        sidelift = ahwMap.dcMotor.get("sidelift");
        //frontlift = ahwMap.dcMotor.get("frontlift");
        //frontliftgrab = ahwMap.crservo.get("frontliftgrab");
        sideliftgrab = ahwMap.servo.get("sideliftgrab");
        platform = ahwMap.servo.get("platform");
        color = ahwMap.get(ColorSensor.class, "cd_sense");
        distance = ahwMap.get(DistanceSensor.class, "cd_sense");


        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sidelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //track.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        four.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //track.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        updateGyro();
    }

    void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }
}