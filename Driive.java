package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.*;
/*import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.PI;*/

class Driive {
    private DcMotor[] wheels;
    private double[] wheelAngles;
    private double zero;
    private double r;
    private double theta;
    private double currentAngle;
    private double setAngle;
    private double turn;
    private boolean turning = false;

    public void setTurnThreshold(double turnThreshold) {
        this.turnThreshold = turnThreshold;
    }

    public void setMinTurnSpeed(double minTurnSpeed) {
        this.minTurnSpeed = minTurnSpeed;
    }

    private double turnThreshold = 0;
    private double minTurnSpeed = 0;

    private double[] wheelPowers;

    private boolean odometry = false;
    private DcMotor[] odometers = new DcMotor[3];
    private double[] odometerPrev = new double[3];
    private static final int LEFT_ENC = 0;
    private static final int CENTER_ENC = 1;
    private static final int RIGHT_ENC = 2;

    private double odometerCPR;
    private double odometerDiameter;
    private double lrDist;
    private double xDist;

    private double odometryX = 0;
    private double odometryY = 0;
    private double odometerRot = 0;

    /**
     * Marks whether the robot's driving is field centric<br>
     * Defaults to true
     */
    boolean fieldCentric = true;

    /**
     * Marks whether the robot fights to maintain its orientation<br>
     * Defaults to false
     */
    boolean righteous = false;

    /**
     * Speed, works on a scale of 1-10<br>
     * Affects the overall speed of the robot
     */
    int speed = 10;

    /**
     * Automatic turning speed
     */
    double turnSpeed = 1;

    /**
     * Initializes the Driive instance<br>
     * Called once in init
     *
     * @param wheels An array of the DcMotors to be used for driving
     * @param wheelAngles An array of the physical angles of the wheels, e.g. 0,pi/2,pi,3pi/2
     * @throws java.lang.ArrayIndexOutOfBoundsException if wheels[] and wheelAngles[] are different lengths
     */
    void init(DcMotor[] wheels, double[] wheelAngles) throws Exception{
        init(wheels, wheelAngles, 0);
    }

    /**
     * Initializes the Driive instance<br>
     * Called once in init
     *
     * @param wheels An array of the DcMotors to be used for driving
     * @param wheelAngles An array of the physical angles of the wheels, e.g. 0,90,180,270
     * @param zero The starting angle of the robot; can be adjusted to change the initial "front"
     * @throws java.lang.Exception - if wheels[] and wheelAngles[] are different lengths
     */
    void init(DcMotor[] wheels, double[] wheelAngles, double zero) throws Exception {
        if(wheels.length != wheelAngles.length) {
            throw new java.lang.Exception("Wheels[] and wheel angles[] are different lengths; please check your init code");
        }
        this.wheels = wheels;
        this.wheelAngles = wheelAngles;
        this.zero = zero;
        this.wheelPowers = new double[wheels.length];
    }

    /**
     * Changes zero power behavior; true is braking, false is floating
     *
     * @param brake
     */
    void setBrake(boolean brake) {
        for(DcMotor wheel:wheels) {
            if(brake) {
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    /**
     * Initializes the odometers<br>
     * Takes left, center, right: | - |
     *
     * @param odometers Array of DcMotors for odometry, NOT ACTUAL MOTORS
     */
    void initOdometry(DcMotor[] odometers) throws Exception {
        if(odometers.length != 3) {
            throw new java.lang.Exception("Must be three DcMotor objects");
        }
        this.odometers = odometers;
        odometry = true;
    }

    /**
     * Returns the absolute position of the robot on the field.<br>
     *
     * @return X, Y, rotation
     */
    double[] getOdometry() {
        double[] arr = {odometryX, odometryY, odometerRot};
        return arr;
    }

    /**
     * Control movement using x,y values<br>
     * Call when movement should change
     *
     * @param x The X value of the joystick
     * @param y The Y value of the joystick
     */
    void cartesian(double x, double y, double turn) {
        r = sqrt(x*x+y*y);
        theta = atan2(x, y);
        this.turn = turn;
    }

    /**
     * Control movement using polar coordinates<br>
     * Call when movement should change
     *
     * @param r The magnitude of the vector
     * @param theta The angle of the vector
     */
    void polar(double r, double theta, double turn) {
        this.r = r;
        this.theta = theta;
        this.turn = turn;
    }

    void polarAuto(double r, double theta, double distance, TeleAuto callback) {
        polarAuto(r, theta, distance, callback, 0, false, false);
        this.r = 0;
        driive();
    }

    void polarAuto(double r, double theta, double distance, TeleAuto callback, double slowclicks) {
        polarAuto(r, theta, distance, callback, slowclicks, true, true);
    }

    /**
     * Drives in a given direction at a given speed for a given distance, using encoders.<br>
     * Blocking function, will not exit until drive is completed. <br>
     * Curves acceleration and deceleration.
     *
     * @param r Speed
     * @param theta Direction in radians from zero
     * @param distance Distance in clicks
     * @param callback this
     * @param slowclicks Number of clicks to ramp out with
     * @param curvein Whether to ramp speed up
     * @param curveout Whether to ramp speed down
     */
    void polarAuto(double r, double theta, double distance, TeleAuto callback, double slowclicks, boolean curvein, boolean curveout) {
        // Records previous preferences
        boolean righteousPrev = righteous;
        boolean fieldCentricPrev = fieldCentric;
        fieldCentric = true;
        righteous = true;

        // Gets initial wheel positions
        double[] wheelDistances = new double[wheels.length];
        for(int i = 0; i < wheels.length; i++) {
            wheelDistances[i] = wheels[i].getCurrentPosition();
        }

        while(callback.opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            // Find the current total average delta
            double total = 0;
            for(int i = 0; i < wheels.length; i++) {
                packet.put("wheel position " + i, wheels[i].getCurrentPosition());
                double currentWheelSin = sin(wheelAngles[i] - theta);
                double wheelDistance = wheels[i].getCurrentPosition() - wheelDistances[i];
                double wheelRobotDistance = currentWheelSin * wheelDistance;
                total += wheelRobotDistance;
            }
            total /= wheels.length;

            // Add data to telemetry
            updateTelemetry(packet);
            packet.put("total", total);
            callback.updateAuto(packet);

            // Drive
            this.turn = 0;
            if(total < (slowclicks / 10) && curvein) {
                this.r = r * total / (slowclicks / 10);
            }
            else if(total > (abs(distance) - slowclicks) && curveout) {
                this.r = r * (distance - total) / slowclicks;
            }
            else {
                this.r = r;
            }
            if(this.r < 0.1) {
                this.r = 0.1;
            }
            //this.r = r;
            this.theta = theta;
            driive();

            // Stop driving
            if (abs(total) > abs(distance) && turn < 0.01) break;
        }

        // Stop driving
        //this.r = 0;
        //driive();

        // Resets to previous values
        fieldCentric = fieldCentricPrev;
        righteous = righteousPrev;
    }

    /**
     * Drives in a given direction at a given speed for a given distance, using encoders.<br>
     * Blocking function, will not exit until drive is completed.
     *
     * @param r Speed
     * @param theta Direction in radians from zero
     * @param callback this
     * @param turn Amount to turn
     */
    void polarAutoTurn(double r, double theta, double delta, TeleAuto callback, double turn) {
        // Records previous preferences
        boolean righteousPrev = righteous;
        boolean fieldCentricPrev = fieldCentric;
        fieldCentric = false;
        righteous = false;

        // Gets initial wheel positions
        /*double[] wheelDistances = new double[wheels.length];
        for(int i = 0; i < wheels.length; i++) {
            wheelDistances[i] = wheels[i].getCurrentPosition();
        }*/

        while(callback.opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            /*// Find the current total average delta
            double total = 0;
            for(int i = 0; i < wheels.length; i++) {
                packet.put("wheel position " + i, wheels[i].getCurrentPosition());
                double currentWheelSin = Math.sin(wheelAngles[i] - theta);
                double wheelDistance = wheels[i].getCurrentPosition() - wheelDistances[i];
                double wheelRobotDistance = currentWheelSin * wheelDistance;
                total += wheelRobotDistance;
            }
            total /= wheels.length;*/

            // Add data to telemetry
            updateTelemetry(packet);
            //packet.put("total", total);
            callback.updateAuto(packet);

            // Drive
            this.turn = turn;
            this.r = r;
            this.theta = theta;
            driive();

            // Stop driving
            if(turn > 0 && wrap(currentAngle - setAngle) > 0) break;
            else if(wrap(currentAngle - setAngle) < 0) break;
        }

        // Stop driving
        this.r = 0;
        driive();

        // Resets to previous values
        fieldCentric = fieldCentricPrev;
        righteous = righteousPrev;
    }

    /**
     * Stops all wheels.
     */
    void stopWheels() {
        for(DcMotor wheel:wheels) {
            wheel.setPower(0);
        }
    }

    /**
     * Sets the gyroscope value<br>
     * Call every loop with updated values
     *
     * @param angle The angle obtained from the gyroscope
     */
    void gyro(double angle) {
        currentAngle = angle;
    }

    /**
     * Resets zero to current position
     */
    void resetZero() {
        zero = -currentAngle;
    }

    /**
     * Turns a specified amount
     *
     * @param delta The amount to turn
     */
    void turnRel(double delta) {
        setAngle = wrap(currentAngle - delta);
        turning = true;
    }

    /**
     * Turns to a specified angle
     *
     * @param angle The angle to turn to
     */
    void turnAbs(double angle) {
        setAngle = zero + angle;
        turning = true;
    }

    /**
     * Does the actual processing and driving<br>
     * Call once per loop
     */
    void driive() {
        // Accounts for rotation using gyro values (if field centric)
        double localtheta = theta;
        if(fieldCentric) {
            localtheta += (currentAngle + zero);
        }

        // Code for automated turning
        if(righteous || turning) {
            // Sets amount to turn based on target position
            if(turn == 0.0) {
                turn = -pow(wrap(currentAngle - setAngle), 3) * turnSpeed;
            }
            // Sets target to current if user is manually turning
            else {
                setAngle = currentAngle;
            }

            // Avoids low power/squeaking motors, if not driving
            if(r != 0) {
                if(abs(turn) < turnThreshold) turn = 0;
                if(turn < minTurnSpeed && turn > turnThreshold) turn = minTurnSpeed;
                if(turn > -minTurnSpeed && turn < -turnThreshold) turn = -minTurnSpeed;
            }
        }

        // Deactivate turning when the turn is done
        if(turning && turn == 0) {
            turning = false;
        }

        // Applies the speed setting
        r *= speed / 10.0;
        turn *= speed / 10.0;

        // Send power to wheels
        for(int i = 0; i != wheels.length; i++) {
            wheels[i].setPower(sin(wheelAngles[i] - localtheta) * r + turn);
            wheelPowers[i] = sin(wheelAngles[i] - localtheta) * r + turn;       // Telemetry
        }

        if(odometry) {
            // find x and y
            double[] odometerDeltas = new double[3];
            for(int i = 0; i < 3; i++) {
                odometerDeltas[i] = odometers[i].getCurrentPosition() - odometerPrev[i];
                odometerPrev[i] = odometers[i].getCurrentPosition();
            }

            //    get average of side encoders
            double odometerAvg = (odometerDeltas[LEFT_ENC] + odometerDeltas[RIGHT_ENC]) / 2;    //clicks
            double robotY = (odometerAvg / odometerCPR) * PI * odometerDiameter;           //mm

            //    get rotation according to encoders
            double odometerDiff = (odometerDeltas[LEFT_ENC] - odometerDeltas[RIGHT_ENC]) / 2;   //clicks
            odometerRot = (odometerDiff / odometerCPR) * PI * odometerDiameter;            //mm
            odometerRot = (2 * odometerRot) / lrDist;                                     //rad

            //    compensate for rotation
            double robotX = (odometerDeltas[CENTER_ENC] / odometerCPR) * PI * odometerDiameter;    //mm
            robotX = robotX - (odometerRot * xDist);                                            //mm?

            // turn into polar coordinate
            double robotrot = atan2(robotX, robotY);
            double robotr = sqrt(robotX*robotX + robotY*robotY);

            // account for gyroscope
            robotrot = wrap(robotrot + currentAngle);

            // turn back into cartesian and add to totals
            odometryX += robotr * cos(robotrot);
            odometryY += robotr * sin(robotrot);
        }
    }

    /**
     * Updates the telemetry packet with driving info
     *
     * @param packet Packet to add data to
     */
    void updateTelemetry(TelemetryPacket packet) {
        packet.put("currentAngle", currentAngle);
        packet.put("zero", zero);
        packet.put("theta", theta);
        packet.put("r", r);
        packet.put("setAngle", setAngle);
        packet.put("turn", turn);
        packet.put("turning", turning);
        packet.put("fieldCentric", fieldCentric);
        packet.put("righteous", righteous);
        for(int i = 0; i != wheelPowers.length; i++) {
            packet.put("wheel power " + i, wheelPowers[i]);
        }
        if(odometry) {
            packet.put("odometryX", odometryX);
            packet.put("odometryY", odometryY);
        }
    }

    private double wrap(double input) {
        while(abs(input) > PI) {
            if(input < -PI) {
                input += 2*PI;
            }
            else {
                input -= 2*PI;
            }
        }
        return input;
    }
}

interface TeleAuto {
    void updateAuto(TelemetryPacket packet);
    boolean opModeIsActive();
}
