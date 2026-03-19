//LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)

package frc.robot;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    public static class LimelightTarget_Retro {
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

        public Pose3d getCameraPose_TargetSpace() { return toPose3D(cameraPose_TargetSpace); }
        public Pose3d getRobotPose_FieldSpace() { return toPose3D(robotPose_FieldSpace); }
        public Pose3d getRobotPose_TargetSpace() { return toPose3D(robotPose_TargetSpace); }
        public Pose3d getTargetPose_CameraSpace() { return toPose3D(targetPose_CameraSpace); }
        public Pose3d getTargetPose_RobotSpace() { return toPose3D(targetPose_RobotSpace); }
        public Pose2d getCameraPose_TargetSpace2D() { return toPose2D(cameraPose_TargetSpace); }
        public Pose2d getRobotPose_FieldSpace2D() { return toPose2D(robotPose_FieldSpace); }
        public Pose2d getRobotPose_TargetSpace2D() { return toPose2D(robotPose_TargetSpace); }
        public Pose2d getTargetPose_CameraSpace2D() { return toPose2D(targetPose_CameraSpace); }
        public Pose2d getTargetPose_RobotSpace2D() { return toPose2D(targetPose_RobotSpace); }

        @JsonProperty("ta") public double ta;
        @JsonProperty("tx") public double tx;
        @JsonProperty("ty") public double ty;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ts") public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTarget_Fiducial {
        @JsonProperty("fID") public double fiducialID;
        @JsonProperty("fam") public String fiducialFamily;
        @JsonProperty("t6c_ts") private double[] cameraPose_TargetSpace;
        @JsonProperty("t6r_fs") private double[] robotPose_FieldSpace;
        @JsonProperty("t6r_ts") private double[] robotPose_TargetSpace;
        @JsonProperty("t6t_cs") private double[] targetPose_CameraSpace;
        @JsonProperty("t6t_rs") private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace() { return toPose3D(cameraPose_TargetSpace); }
        public Pose3d getRobotPose_FieldSpace() { return toPose3D(robotPose_FieldSpace); }
        public Pose3d getRobotPose_TargetSpace() { return toPose3D(robotPose_TargetSpace); }
        public Pose3d getTargetPose_CameraSpace() { return toPose3D(targetPose_CameraSpace); }
        public Pose3d getTargetPose_RobotSpace() { return toPose3D(targetPose_RobotSpace); }
        public Pose2d getCameraPose_TargetSpace2D() { return toPose2D(cameraPose_TargetSpace); }
        public Pose2d getRobotPose_FieldSpace2D() { return toPose2D(robotPose_FieldSpace); }
        public Pose2d getRobotPose_TargetSpace2D() { return toPose2D(robotPose_TargetSpace); }
        public Pose2d getTargetPose_CameraSpace2D() { return toPose2D(targetPose_CameraSpace); }
        public Pose2d getTargetPose_RobotSpace2D() { return toPose2D(targetPose_RobotSpace); }

        @JsonProperty("ta") public double ta;
        @JsonProperty("tx") public double tx;
        @JsonProperty("ty") public double ty;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ts") public double ts;

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTarget_Barcode {
        @JsonProperty("fam") public String family;
        @JsonProperty("data") public String data;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx") public double tx;
        @JsonProperty("ty") public double ty;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ta") public double ta;
        @JsonProperty("pts") public double[][] corners;

        public LimelightTarget_Barcode() {}
        public String getFamily() { return family; }
    }

    public static class LimelightTarget_Classifier {
        @JsonProperty("class") public String className;
        @JsonProperty("classID") public double classID;
        @JsonProperty("conf") public double confidence;
        @JsonProperty("zone") public double zone;
        @JsonProperty("tx") public double tx;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("ty") public double ty;
        @JsonProperty("typ") public double ty_pixels;
        public LimelightTarget_Classifier() {}
    }

    public static class LimelightTarget_Detector {
        @JsonProperty("class") public String className;
        @JsonProperty("classID") public double classID;
        @JsonProperty("conf") public double confidence;
        @JsonProperty("ta") public double ta;
        @JsonProperty("tx") public double tx;
        @JsonProperty("ty") public double ty;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        public LimelightTarget_Detector() {}
    }

    public static class LimelightResults {
        public String error;
        @JsonProperty("pID") public double pipelineID;
        @JsonProperty("tl") public double latency_pipeline;
        @JsonProperty("cl") public double latency_capture;
        public double latency_jsonParse;
        @JsonProperty("ts") public double timestamp_LIMELIGHT_publish;
        @JsonProperty("ts_rio") public double timestamp_RIOFPGA_capture;
        @JsonProperty("v") @JsonFormat(shape = Shape.NUMBER) public boolean valid;
        @JsonProperty("botpose") public double[] botpose;
        @JsonProperty("botpose_wpired") public double[] botpose_wpired;
        @JsonProperty("botpose_wpiblue") public double[] botpose_wpiblue;
        @JsonProperty("botpose_tagcount") public double botpose_tagcount;
        @JsonProperty("botpose_span") public double botpose_span;
        @JsonProperty("botpose_avgdist") public double botpose_avgdist;
        @JsonProperty("botpose_avgarea") public double botpose_avgarea;
        @JsonProperty("t6c_rs") public double[] camerapose_robotspace;
        @JsonProperty("Retro") public LimelightTarget_Retro[] targets_Retro;
        @JsonProperty("Fiducial") public LimelightTarget_Fiducial[] targets_Fiducials;
        @JsonProperty("Classifier") public LimelightTarget_Classifier[] targets_Classifier;
        @JsonProperty("Detector") public LimelightTarget_Detector[] targets_Detector;
        @JsonProperty("Barcode") public LimelightTarget_Barcode[] targets_Barcode;

        public Pose3d getBotPose3d() { return toPose3D(botpose); }
        public Pose3d getBotPose3d_wpiRed() { return toPose3D(botpose_wpired); }
        public Pose3d getBotPose3d_wpiBlue() { return toPose3D(botpose_wpiblue); }
        public Pose2d getBotPose2d() { return toPose2D(botpose); }
        public Pose2d getBotPose2d_wpiRed() { return toPose2D(botpose_wpired); }
        public Pose2d getBotPose2d_wpiBlue() { return toPose2D(botpose_wpiblue); }

        public LimelightResults() {
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

    public static class RawFiducial {
        public int id = 0;
        public double txnc = 0;
        public double tync = 0;
        public double ta = 0;
        public double distToCamera = 0;
        public double distToRobot = 0;
        public double ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta, double distToCamera, double distToRobot, double ambiguity) {
            this.id = id;
            this.txnc = txnc;
            this.tync = tync;
            this.ta = ta;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
        }
    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;
        public RawFiducial[] rawFiducials;
        public boolean isMegaTag2;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
            this.rawFiducials = new RawFiducial[]{};
            this.isMegaTag2 = false;
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                           int tagCount, double tagSpan, double avgTagDist,
                           double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
            this.rawFiducials = rawFiducials;
            this.isMegaTag2 = isMegaTag2;
        }
    }

    private static ObjectMapper mapper;
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if ("".equals(name) || name == null) {
            return "limelight";
        }
        return name;
    }

    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;

        if (poseArray.length == 0) {
            return new PoseEstimate();
        }

        var pose = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int tagCount = (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);

        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        RawFiducial[] rawFiducials;

        if (poseArray.length != expectedTotalVals) {
            rawFiducials = new RawFiducial[0];
        } else {
            rawFiducials = new RawFiducial[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int) poseArray[baseIndex];
                double txnc = poseArray[baseIndex + 1];
                double tync = poseArray[baseIndex + 2];
                double ta = poseArray[baseIndex + 3];
                double distToCamera = poseArray[baseIndex + 4];
                double distToRobot = poseArray[baseIndex + 5];
                double ambiguity = poseArray[baseIndex + 6];
                rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
            }
        }

        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    public static Boolean validPoseEstimate(PoseEstimate pose) {
        return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k -> {
            NetworkTable table = getLimelightNTTable(tableName);
            return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
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

    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
    }

    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
    }

    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired", false);
    }

    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public static void SetRobotOrientation(String limelightName, double yaw, double yawRate,
                                           double pitch, double pitchRate,
                                           double roll, double rollRate) {
        double[] entries = new double[6];
        entries[0] = yaw;
        entries[1] = yawRate;
        entries[2] = pitch;
        entries[3] = pitchRate;
        entries[4] = roll;
        entries[5] = rollRate;
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries);
        Flush();
    }

    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side, double up,
                                                 double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    public static LimelightResults getLatestResults(String limelightName) {
        long start = System.nanoTime();
        LimelightResults results = new LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            String jsonString = getLimelightNTString(limelightName, "json");
            if (jsonString == null || jsonString.isEmpty()) {
                results.error = "lljson error: empty json";
            } else {
                results = mapper.readValue(jsonString, LimelightResults.class);
            }
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.latency_jsonParse = millis;

        return results;
    }
}
