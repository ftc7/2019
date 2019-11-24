package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    CRServo leftintake = null;
    CRServo rightintake = null;
    DcMotor track = null;
    DcMotor sidelift = null;
    DcMotor frontlift = null;
    CRServo frontliftgrab = null;
    Servo sideliftgrab = null;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    float[] dangles = new float[3];

    HardwareMap hwMap = null;

    Blinky(){}

    void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        one = ahwMap.dcMotor.get("one");
        two = ahwMap.dcMotor.get("two");
        three = ahwMap.dcMotor.get("three");
        four = ahwMap.dcMotor.get("four");

        leftintake = ahwMap.crservo.get("leftintake");
        rightintake = ahwMap.crservo.get("rightintake");
        track = ahwMap.dcMotor.get("track");
        sidelift = ahwMap.dcMotor.get("sidelift");
        frontlift = ahwMap.dcMotor.get("frontlift");
        frontliftgrab = ahwMap.crservo.get("frontliftgrab");
        sideliftgrab = ahwMap.servo.get("sideliftgrab");

        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sidelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        track.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        dangles[0] = angles.firstAngle;
        dangles[1] = angles.secondAngle;
        dangles[2] = angles.thirdAngle;
    }

    void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        /*dangles[0] = ((dangles[0] * damping) + angles.firstAngle) / (damping + 1);
        dangles[1] = ((dangles[1] * damping) + angles.secondAngle) / (damping + 1);
        dangles[2] = ((dangles[2] * damping) + angles.thirdAngle) / (damping + 1);*/

        gravity = imu.getGravity();
    }
}