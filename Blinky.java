package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

class Blinky {
    DcMotor one = null;
    DcMotor two = null;
    DcMotor three = null;
    DcMotor four = null;

    DcMotor leftintake = null;
    DcMotor rightintake = null;
    DcMotor tape = null;
    DcMotor sidelift = null;
    Servo sideliftgrab = null;
    Servo platform = null;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    HardwareMap hwMap = null;

    ColorSensor color;
    DistanceSensor distance_alignment;
    DistanceSensor distance_platform;
    DistanceSensor distance_unplat;
    DistanceSensor distance_blockplat;

    DigitalChannel intake_button;

    Blinky(){}

    void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        one = ahwMap.dcMotor.get("one");
        two = ahwMap.dcMotor.get("two");
        three = ahwMap.dcMotor.get("three");
        four = ahwMap.dcMotor.get("four");
        leftintake = ahwMap.dcMotor.get("leftintake");
        rightintake = ahwMap.dcMotor.get("rightintake");
        tape = ahwMap.dcMotor.get("tape");

        sidelift = ahwMap.dcMotor.get("sidelift");
        sideliftgrab = ahwMap.servo.get("sideliftgrab");
        platform = ahwMap.servo.get("platform");
        color = ahwMap.get(ColorSensor.class, "cd_sense");
        distance_alignment = ahwMap.get(DistanceSensor.class, "cd_sense");
        distance_platform = ahwMap.get(DistanceSensor.class, "platform_sense");
        distance_unplat = ahwMap.get(DistanceSensor.class, "distance_unplat");
        distance_blockplat = ahwMap.get(DistanceSensor.class, "distance_blockplat");

        intake_button = ahwMap.get(DigitalChannel.class, "intake_button");
        intake_button.setMode(DigitalChannel.Mode.INPUT);

        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightintake.setDirection(DcMotorSimple.Direction.REVERSE);
        tape.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sidelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        three.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        four.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}