package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

class gpPrev1 {
    float left_stick_x = 0f;
    float left_stick_y = 0f;
    float right_stick_x = 0f;
    float right_stick_y = 0f;
    boolean dpad_up = false;
    boolean dpad_down = false;
    boolean dpad_left = false;
    boolean dpad_right = false;
    boolean a = false;
    boolean b = false;
    boolean x = false;
    boolean y = false;
    boolean start = false;
    boolean back = false;
    boolean guide = false;

    void update(Gamepad gamepad) {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        start = gamepad.start;
        back = gamepad.back;
        guide = gamepad.guide;
    }
}

@Config
class config {
    //public static int servoPower = 0;
}

@TeleOp(name = "Driive v2")
public class DriiveTeleop extends OpMode {
    private DriivePrototypeHardware robot = new DriivePrototypeHardware();
    //private gpPrev1 prev1 = new gpPrev1();
    private Gamepad prev1 = new Gamepad();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();

    private double angle1 = 0.0;
    private double currentAngle = 0.0;

    public void init() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double pi = Math.PI;
        double[] angles = {pi/4, pi*3/4, pi*5/4, pi*7/4};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }
        telemetry.addData("Initialized", "Initialized");
        telemetry.update();
    }

    public void init_loop() {
        driving.gyro(Math.toRadians(-robot.angles.thirdAngle));
        TelemetryPacket packet = new TelemetryPacket();
        driving.updateTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
        currentAngle = Math.toRadians(-robot.angles.thirdAngle);

        // -- DRIVING --

        robot.updateGyro(0);

        driving.cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        driving.gyro(currentAngle);
        driving.driive();

        // -- CONTROLS --

        if(gamepad1.left_bumper) {
            angle1 = currentAngle;
        }
        if(gamepad1.right_bumper) {
            driving.turnAbs(angle1);
        }

        if(gamepad1.x && !prev1.x) {
            driving.fieldCentric = !driving.fieldCentric;
        }
        if(gamepad1.start && !prev1.start) {
            driving.resetZero();
        }

        //prev1.update(gamepad1);
        try {
            prev1.copy(gamepad1);
        } catch(RobotCoreException e) {}

        // -- TELEMETRY --

        TelemetryPacket packet =  new TelemetryPacket();

        packet.put("fieldCentric", driving.fieldCentric);
        packet.put("righteous", driving.righteous);
        packet.put("speed", driving.speed);

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);

        // -- TESTING MOTORS --

        if(gamepad1.left_bumper) {
            robot.servo.setPower(1);
            robot.servo1.setPower(-1);
        } else if(gamepad1.right_bumper) {
            robot.servo.setPower(-1);
            robot.servo1.setPower(1);
        } else {
            robot.servo.setPower(0);
            robot.servo1.setPower(0);
        }
    }
}