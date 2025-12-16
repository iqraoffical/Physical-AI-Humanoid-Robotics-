using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2Sharp;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Nav_msgs;

public class UnityRosConnector : MonoBehaviour
{
    // Connection configuration
    [Header("ROS Connection Settings")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 5005;
    
    // Robot model reference
    [Header("Robot Model")]
    public GameObject robotModel;
    
    // Sensor visualization objects
    [Header("Sensor Visualization")]
    public GameObject lidarVisualization;
    public GameObject depthCameraVisualization;
    public GameObject imuIndicator;
    
    // Robot joint references (for 24+ DOF humanoid)
    [Header("Joint References")]
    public Transform headJoint;
    public Transform[] leftArmJoints;   // 6 joints for left arm
    public Transform[] rightArmJoints;  // 6 joints for right arm
    public Transform[] leftLegJoints;   // 6 joints for left leg
    public Transform[] rightLegJoints;  // 6+ joints for right leg
    
    // Data storage
    private ROSConnection rosConnection;
    private JointStateMsg currentJointState;
    private OdometryMsg currentOdometry;
    private LaserScanMsg currentLidarData;
    private ImuMsg currentImuData;
    
    // Timing and synchronization
    private float lastUpdateTime;
    private float syncFrequency = 60.0f;  // Sync at 60 Hz as per requirements
    
    // Use this for initialization
    void Start()
    {
        // Initialize ROS connection
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Connect(rosIPAddress, rosPort);
        
        // Subscribe to topics
        rosConnection.Subscribe<JointStateMsg>("/joint_states", JointStateCallback);
        rosConnection.Subscribe<OdometryMsg>("/odom", OdometryCallback);
        rosConnection.Subscribe<LaserScanMsg>("/sensors/lidar/scan", LidarCallback);
        rosConnection.Subscribe<ImuMsg>("/sensors/imu/data", ImuCallback);
        
        // Initialize timing
        lastUpdateTime = Time.time;
        
        Debug.Log("Unity ROS Connector initialized and subscribed to topics");
    }
    
    // Update is called once per frame
    void Update()
    {
        // Synchronize with simulation at specified frequency
        float timeSinceLastUpdate = Time.time - lastUpdateTime;
        float updateTimeThreshold = 1.0f / syncFrequency;
        
        if (timeSinceLastUpdate >= updateTimeThreshold)
        {
            UpdateRobotVisualization();
            UpdateSensorVisualization();
            lastUpdateTime = Time.time;
        }
    }
    
    // Callback for joint state messages
    void JointStateCallback(JointStateMsg jointStateMsg)
    {
        // Store the latest joint state
        this.currentJointState = jointStateMsg;
        
        Debug.Log($"Received joint state with {jointStateMsg.name.Count} joints");
    }
    
    // Callback for odometry messages
    void OdometryCallback(OdometryMsg odometryMsg)
    {
        // Store the latest odometry data
        this.currentOdometry = odometryMsg;
        
        // Update robot position and orientation in Unity
        UpdateRobotPosition(odometryMsg);
    }
    
    // Callback for LiDAR messages
    void LidarCallback(LaserScanMsg lidarMsg)
    {
        // Store the latest LiDAR data
        this.currentLidarData = lidarMsg;
    }
    
    // Callback for IMU messages
    void ImuCallback(ImuMsg imuMsg)
    {
        // Store the latest IMU data
        this.currentImuData = imuMsg;
        
        // Update IMU indicator in visualization
        UpdateImuIndicator(imuMsg);
    }
    
    // Update robot position and orientation based on odometry
    void UpdateRobotPosition(OdometryMsg odometryMsg)
    {
        if (robotModel != null)
        {
            // Convert ROS pose to Unity coordinates (ROS uses right-handed, Unity uses left-handed)
            Vector3 position = new Vector3(
                (float)odometryMsg.pose.pose.position.x,
                (float)odometryMsg.pose.pose.position.z,  // Switch Y and Z
                (float)odometryMsg.pose.pose.position.y   // Switch Y and Z
            );
            
            // Convert quaternion from ROS to Unity (account for coordinate system differences)
            Quaternion orientation = new Quaternion(
                -(float)odometryMsg.pose.pose.orientation.y,  // Negate and switch
                -(float)odometryMsg.pose.pose.orientation.z,  // Negate and switch
                (float)odometryMsg.pose.pose.orientation.x,   // Switch
                (float)odometryMsg.pose.pose.orientation.w
            );
            
            robotModel.transform.position = position;
            robotModel.transform.rotation = orientation;
        }
    }
    
    // Update robot joint positions based on joint states
    void UpdateRobotVisualization()
    {
        if (currentJointState != null && robotModel != null)
        {
            // Update head joint
            UpdateJoint(headJoint, "head_joint", currentJointState);
            
            // Update left arm joints (6 joints)
            if (leftArmJoints != null && leftArmJoints.Length >= 6)
            {
                UpdateJoint(leftArmJoints[0], "left_shoulder_yaw", currentJointState);
                UpdateJoint(leftArmJoints[1], "left_shoulder_pitch", currentJointState);
                UpdateJoint(leftArmJoints[2], "left_shoulder_roll", currentJointState);
                UpdateJoint(leftArmJoints[3], "left_elbow", currentJointState);
                UpdateJoint(leftArmJoints[4], "left_wrist_yaw", currentJointState);
                UpdateJoint(leftArmJoints[5], "left_wrist_pitch", currentJointState);
            }
            
            // Update right arm joints (6 joints)
            if (rightArmJoints != null && rightArmJoints.Length >= 6)
            {
                UpdateJoint(rightArmJoints[0], "right_shoulder_yaw", currentJointState);
                UpdateJoint(rightArmJoints[1], "right_shoulder_pitch", currentJointState);
                UpdateJoint(rightArmJoints[2], "right_shoulder_roll", currentJointState);
                UpdateJoint(rightArmJoints[3], "right_elbow", currentJointState);
                UpdateJoint(rightArmJoints[4], "right_wrist_yaw", currentJointState);
                UpdateJoint(rightArmJoints[5], "right_wrist_pitch", currentJointState);
            }
            
            // Update left leg joints (6+ joints)
            if (leftLegJoints != null && leftLegJoints.Length >= 6)
            {
                UpdateJoint(leftLegJoints[0], "left_hip_yaw", currentJointState);
                UpdateJoint(leftLegJoints[1], "left_hip_roll", currentJointState);
                UpdateJoint(leftLegJoints[2], "left_hip_pitch", currentJointState);
                UpdateJoint(leftLegJoints[3], "left_knee", currentJointState);
                UpdateJoint(leftLegJoints[4], "left_ankle_yaw", currentJointState);
                UpdateJoint(leftLegJoints[5], "left_ankle_pitch", currentJointState);
            }
            
            // Update right leg joints (6+ joints)
            if (rightLegJoints != null && rightLegJoints.Length >= 6)
            {
                UpdateJoint(rightLegJoints[0], "right_hip_yaw", currentJointState);
                UpdateJoint(rightLegJoints[1], "right_hip_roll", currentJointState);
                UpdateJoint(rightLegJoints[2], "right_hip_pitch", currentJointState);
                UpdateJoint(rightLegJoints[3], "right_knee", currentJointState);
                UpdateJoint(rightLegJoints[4], "right_ankle_yaw", currentJointState);
                UpdateJoint(rightLegJoints[5], "right_ankle_pitch", currentJointState);
            }
        }
    }
    
    // Helper function to update a specific joint
    void UpdateJoint(Transform jointTransform, string jointName, JointStateMsg jointState)
    {
        if (jointTransform == null || jointState.name == null) 
            return;
        
        // Find the index of the joint in the joint state message
        for (int i = 0; i < jointState.name.Count; i++)
        {
            if (jointState.name[i] == jointName && i < jointState.position.Count)
            {
                // Apply rotation based on joint position (convert radians to degrees)
                float angle = (float)(jointState.position[i] * Mathf.Rad2Deg);
                
                // Apply rotation differently based on joint type
                // For simplicity, assuming all joints are revolute and rotate around Y axis
                jointTransform.localRotation = Quaternion.Euler(0, angle, 0);
                break;
            }
        }
    }
    
    // Update sensor visualizations
    void UpdateSensorVisualization()
    {
        if (currentLidarData != null)
        {
            UpdateLidarVisualization();
        }
        
        if (currentImuData != null)
        {
            UpdateImuIndicator(currentImuData);
        }
    }
    
    // Update LiDAR visualization based on LiDAR data
    void UpdateLidarVisualization()
    {
        if (lidarVisualization != null && currentLidarData.ranges != null)
        {
            // This would create or update LiDAR point visualization
            // For simplicity, just log the info
            Debug.Log($"Updating LiDAR visualization with {currentLidarData.ranges.Count} points");
            
            // In a real implementation, we would create/clear point objects based on LiDAR data
            // and position them according to the ranges and angle information
        }
    }
    
    // Update IMU indicator based on IMU data
    void UpdateImuIndicator(ImuMsg imuMsg)
    {
        if (imuIndicator != null)
        {
            // Update the IMU indicator based on orientation data
            Quaternion imuRotation = new Quaternion(
                (float)imuMsg.orientation.x,
                (float)imuMsg.orientation.y,
                (float)imuMsg.orientation.z,
                (float)imuMsg.orientation.w
            );
            
            imuIndicator.transform.rotation = imuRotation;
        }
    }
    
    // Publish joint commands to ROS
    public void SendJointCommands(Dictionary<string, double> jointCommands)
    {
        JointStateMsg jointCommand = new JointStateMsg();
        jointCommand.name = new List<string>();
        jointCommand.position = new List<double>();
        
        foreach (var kvp in jointCommands)
        {
            jointCommand.name.Add(kvp.Key);
            jointCommand.position.Add(kvp.Value);
        }
        
        rosConnection.Publish(jointCommand, "/joint_commands");
    }
    
    // Function to send commands to the robot
    public void SendRobotCommand(double[] positions)
    {
        // Example of sending a command to the robot
        // This would implement the command interface for the humanoid robot
        Debug.Log("Sending robot command: " + string.Join(", ", positions));
    }
    
    // Function to get current robot state
    public Dictionary<string, double> GetCurrentJointPositions()
    {
        Dictionary<string, double> jointPositions = new Dictionary<string, double>();
        
        if (currentJointState != null)
        {
            for (int i = 0; i < currentJointState.name.Count; i++)
            {
                if (i < currentJointState.position.Count)
                {
                    jointPositions[currentJointState.name[i]] = currentJointState.position[i];
                }
            }
        }
        
        return jointPositions;
    }
    
    // On application quit, disconnect from ROS
    void OnApplicationQuit()
    {
        if (rosConnection != null)
        {
            rosConnection.Disconnect();
        }
    }
}