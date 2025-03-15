//LimelightHelpers v1.2.1 (March 1, 2023)

package frc.utils;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {

    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private  double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }
        
        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;
        
        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTarget_Barcode {

    }

    public static class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public  LimelightTarget_Classifier() {
        }
    }

    public static class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Detector() {
        }
    }

    public static class Results {

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_Limelight_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }
    
        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }
    
        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }
    
        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }
    
        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public Results() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];

        }
    }

    public static class LimelightResults {
        @JsonProperty("Results")
        public Results targetingResults;

        public LimelightResults() {
            targetingResults = new Results();
        }
    }

    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "Limelight";
        }
        return name;
    }

    private static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    private static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public static URL getLimelightURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    public static double getTX(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "tx");
    }

    public static double getTY(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "ty");
    }

    public static double getTA(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "ta");
    }

    public static double getLatency_Pipeline(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "tl");
    }

    public static double getLatency_Capture(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "cl");
    }

    public static double getCurrentPipelineIndex(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "getpipe");
    }

    public static String getJSONDump(String LimelightName) {
        return getLimelightNTString(LimelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param LimelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param LimelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param LimelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "targetpose_robotspace");
    }

    public static double[] getTargetColor(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "tc");
    }

    public static double getFiducialID(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "tid");
    }

    public static double getNeuralClassID(String LimelightName) {
        return getLimelightNTDouble(LimelightName, "tclass");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "botpose");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiRed(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiBlue(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_TargetSpace(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_TargetSpace(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_CameraSpace(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_RobotSpace(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_RobotSpace(String LimelightName) {
        double[] poseArray = getLimelightNTDoubleArray(LimelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param LimelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiBlue(String LimelightName) {

        double[] result = getBotPose_wpiBlue(LimelightName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param LimelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiRed(String LimelightName) {

        double[] result = getBotPose_wpiRed(LimelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param LimelightName
     * @return
     */
    public static Pose2d getBotPose2d(String LimelightName) {

        double[] result = getBotPose(LimelightName);
        return toPose2D(result);

    }

    public static boolean getTV(String LimelightName) {
        return 1.0 == getLimelightNTDouble(LimelightName, "tv");
    }

    /////
    /////

    public static void setPipelineIndex(String LimelightName, int pipelineIndex) {
        setLimelightNTDouble(LimelightName, "pipeline", pipelineIndex);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public static void setLEDMode_PipelineControl(String LimelightName) {
        setLimelightNTDouble(LimelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String LimelightName) {
        setLimelightNTDouble(LimelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String LimelightName) {
        setLimelightNTDouble(LimelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String LimelightName) {
        setLimelightNTDouble(LimelightName, "ledMode", 3);
    }

    public static void setStreamMode_Standard(String LimelightName) {
        setLimelightNTDouble(LimelightName, "stream", 0);
    }

    public static void setStreamMode_PiPMain(String LimelightName) {
        setLimelightNTDouble(LimelightName, "stream", 1);
    }

    public static void setStreamMode_PiPSecondary(String LimelightName) {
        setLimelightNTDouble(LimelightName, "stream", 2);
    }

    public static void setCameraMode_Processor(String LimelightName) {
        setLimelightNTDouble(LimelightName, "camMode", 0);
    }
    public static void setCameraMode_Driver(String LimelightName) {
        setLimelightNTDouble(LimelightName, "camMode", 1);
    }


    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public static void setCropWindow(String LimelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray(LimelightName, "crop", entries);
    }

    public static void setCameraPose_RobotSpace(String LimelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(LimelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public static void setPythonScriptData(String LimelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(LimelightName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String LimelightName) {
        return getLimelightNTDoubleArray(LimelightName, "llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
    }

    private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getLimelightURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object
     */
    public static LimelightResults getLatestResults(String LimelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(LimelightName), LimelightResults.class);
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.targetingResults.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}