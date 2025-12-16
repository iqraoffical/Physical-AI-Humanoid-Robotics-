using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2Sharp;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;
using System.Threading.Tasks;

public class RobotController : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "humanoid_robot";
    public float moveSpeed = 1.0f;
    public float turnSpeed = 90.0f; // degrees per second
    
    [Header("Joint Mapping")]
    public JointMapping[] jointMappings;
    
    [Header("ROS Settings")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 5005;
    
    // Internal references
    private ROSConnection ros;
    private Dictionary<string, float> currentJointPositions;
    private Dictionary<string, float> targetJointPositions;
    private List<GameObject> lidarPoints;  // For LiDAR visualization
    private LineRenderer lidarLineRenderer;  // Alternative visualization
    
    // Performance tracking
    private float syncFrequency = 60.0f;  // Hz - from requirements
    private float maxDesyncThreshold = 0.1f;  // seconds - from requirements
    private float lastSyncTime;
    
    // Robot model references (24+ DOF humanoid)
    [Header("Humanoid Joint Transforms")]
    public Transform head;
    public Transform leftShoulder;
    public Transform leftElbow;
    public Transform leftWrist;
    public Transform rightShoulder;
    public Transform rightElbow;
    public Transform rightWrist;
    public Transform leftHip;
    public Transform leftKnee;
    public Transform leftAnkle;
    public Transform rightHip;
    public Transform rightKnee;
    public Transform rightAnkle;
    
    // Sensor data storage
    private float[] lidarReadings = new float[360];  // For 360-degree LiDAR
    private Vector3 imuOrientation = Vector3.zero;
    private Texture2D depthTexture;
    
    // For visualization
    private GameObject[] lidarPoints;
    private int maxLidarPoints = 360;
    private Color lidarColor = Color.red;
    
    // Robot root for movement
    public Transform robotRoot;
    
    // Animation controller
    private Animator animator;
    private bool isWalking = false;
    
    // Timing for synchronization
    private float syncTimer = 0f;
    private float syncInterval = 0.0167f; // ~60 Hz sync rate
    
    // For movement
    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private bool hasNewTarget = false;

    // Initialize the robot controller
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect(rosIP, rosPort);
        
        // Initialize joint dictionaries
        currentJointPositions = new Dictionary<string, float>();
        targetJointPositions = new Dictionary<string, float>();
        
        // Initialize LiDAR visualization
        InitializeLidarVisualization();
        
        // Initialize depth camera texture
        InitializeDepthTexture();
        
        // Setup ROS subscriptions
        ros.Subscribe<JointStateMsg>("/joint_states", JointStateCallback);
        ros.Subscribe<OdomMsg>("/odom", OdometryCallback);
        ros.Subscribe<LaserScanMsg>("/sensors/lidar/scan", LidarCallback);
        ros.Subscribe<ImuMsg>("/sensors/imu/data", ImuCallback);
        
        // Initialize robot transforms if not set
        if (robotRoot == null)
            robotRoot = transform; // Use this transform as root if none specified
        
        // Get animator component if it exists
        animator = GetComponent<Animator>();
        
        // Initialize default joint positions
        InitializeDefaultJointPositions();
        
        Debug.Log("RobotController initialized with " + jointMappings.Length + " joint mappings");
    }
    
    void InitializeDefaultJointPositions()
    {
        // Initialize all joints to 0 position as default
        // This would be expanded in a real implementation to match home position
        string[] jointNames = {
            "head_yaw_joint", "head_pitch_joint",
            "left_shoulder_yaw", "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_joint", "left_wrist_yaw", "left_wrist_pitch",
            "right_shoulder_yaw", "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_joint", "right_wrist_yaw", "right_wrist_pitch",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_yaw", "left_ankle_pitch",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_yaw", "right_ankle_pitch"
        };
        
        foreach (string jointName in jointNames)
        {
            currentJointPositions[jointName] = 0f;
            targetJointPositions[jointName] = 0f;
        }
    }
    
    // Update is called once per frame
    void Update()
    {
        // Process incoming ROS messages
        ros.Process();
        
        // Update robot visualization based on joint states
        UpdateRobotJoints();
        
        // Update sensor visualizations
        UpdateSensorVisualizations();
        
        // Handle robot movement if target is set
        UpdateRobotMovement();
        
        // Periodically synchronize with simulation (based on sync frequency requirement)
        syncTimer += Time.deltaTime;
        if (syncTimer >= syncInterval)
        {
            SynchronizeWithSimulation();
            syncTimer = 0f;
        }
    }
    
    // Callback for joint state messages from ROS
    void JointStateCallback(JointStateMsg jointStateMsg)
    {
        if (jointStateMsg.name.Count != jointStateMsg.position.Count)
        {
            Debug.LogWarning("Joint state message has mismatched name/position arrays");
            return;
        }
        
        // Update our local joint position storage
        for (int i = 0; i < jointStateMsg.name.Count; i++)
        {
            string jointName = jointStateMsg.name[i];
            float position = (float)jointStateMsg.position[i];
            
            if (currentJointPositions.ContainsKey(jointName))
            {
                currentJointPositions[jointName] = position;
            }
            else
            {
                // Add new joint if not already tracked
                currentJointPositions[jointName] = position;
                targetJointPositions[jointName] = position; // Initialize target to current
            }
        }
        
        // Update joint mapping transforms
        UpdateJointMappingTransforms();
    }
    
    // Callback for odometry messages
    void OdometryCallback(OdometryMsg odomMsg)
    {
        // Extract position and orientation from odometry message
        Vector3 position = new Vector3(
            (float)odomMsg.pose.pose.position.x,
            (float)odomMsg.pose.pose.position.z,  // Convert from right-handed to left-handed
            (float)odomMsg.pose.pose.position.y
        );
        
        // Convert quaternion orientation
        Quaternion orientation = new Quaternion(
            -(float)odomMsg.pose.pose.orientation.y,
            -(float)odomMsg.pose.pose.orientation.z,
            (float)odomMsg.pose.pose.orientation.x,
            (float)odomMsg.pose.pose.orientation.w
        );
        
        // Apply the pose to the robot root
        if (robotRoot != null)
        {
            robotRoot.position = position;
            robotRoot.rotation = orientation;
        }
    }
    
    // Callback for LiDAR messages
    void LidarCallback(LaserScanMsg lidarMsg)
    {
        // Store LiDAR data for visualization
        if (lidarMsg.ranges.Count > 0)
        {
            // Copy the range data
            for (int i = 0; i < Mathf.Min(lidarMsg.ranges.Count, lidarReadings.Length); i++)
            {
                double range = lidarMsg.ranges[i];
                lidarReadings[i] = range < lidarMsg.range_max ? (float)range : Mathf.Infinity;
            }
        }
        
        // Update LiDAR visualization
        UpdateLidarVisualization();
    }
    
    // Callback for IMU messages
    void ImuCallback(ImuMsg imuMsg)
    {
        // Store IMU orientation data
        imuOrientation = new Vector3(
            (float)imuMsg.orientation.x,
            (float)imuMsg.orientation.y,
            (float)imuMsg.orientation.z
        );
        
        // Update IMU visualization
        UpdateImuVisualization();
    }
    
    // Update robot joint transforms based on joint positions
    void UpdateRobotJoints()
    {
        // Head joints
        if (head != null)
        {
            if (currentJointPositions.ContainsKey("head_yaw_joint"))
                head.localEulerAngles = new Vector3(0, currentJointPositions["head_yaw_joint"] * Mathf.Rad2Deg, 0);
                
            if (currentJointPositions.ContainsKey("head_pitch_joint"))
                head.Rotate(currentJointPositions["head_pitch_joint"] * Mathf.Rad2Deg, 0, 0);
        }
        
        // Left arm joints
        if (leftShoulder != null && currentJointPositions.ContainsKey("left_shoulder_yaw"))
            leftShoulder.localEulerAngles = new Vector3(0, currentJointPositions["left_shoulder_yaw"] * Mathf.Rad2Deg, 0);
        if (leftElbow != null && currentJointPositions.ContainsKey("left_elbow_joint"))
            leftElbow.localEulerAngles = new Vector3(0, 0, currentJointPositions["left_elbow_joint"] * Mathf.Rad2Deg);
            
        // Right arm joints
        if (rightShoulder != null && currentJointPositions.ContainsKey("right_shoulder_yaw"))
            rightShoulder.localEulerAngles = new Vector3(0, currentJointPositions["right_shoulder_yaw"] * Mathf.Rad2Deg, 0);
        if (rightElbow != null && currentJointPositions.ContainsKey("right_elbow_joint"))
            rightElbow.localEulerAngles = new Vector3(0, 0, currentJointPositions["right_elbow_joint"] * Mathf.Rad2Deg);
            
        // Left leg joints
        if (leftHip != null && currentJointPositions.ContainsKey("left_hip_yaw"))
            leftHip.localEulerAngles = new Vector3(0, currentJointPositions["left_hip_yaw"] * Mathf.Rad2Deg, 0);
        if (leftKnee != null && currentJointPositions.ContainsKey("left_knee"))
            leftKnee.localEulerAngles = new Vector3(0, 0, currentJointPositions["left_knee"] * Mathf.Rad2Deg);
            
        // Right leg joints
        if (rightHip != null && currentJointPositions.ContainsKey("right_hip_yaw"))
            rightHip.localEulerAngles = new Vector3(0, currentJointPositions["right_hip_yaw"] * Mathf.Rad2Deg, 0);
        if (rightKnee != null && currentJointPositions.ContainsKey("right_knee"))
            rightKnee.localEulerAngles = new Vector3(0, 0, currentJointPositions["right_knee"] * Mathf.Rad2Deg);
    }
    
    // Update joint mapping transforms based on joint state message
    void UpdateJointMappingTransforms()
    {
        foreach (JointMapping mapping in jointMappings)
        {
            if (mapping.jointTransform != null && currentJointPositions.ContainsKey(mapping.jointName))
            {
                float angle = currentJointPositions[mapping.jointName] * Mathf.Rad2Deg;
                
                // Apply rotation based on joint axis
                switch (mapping.rotationAxis)
                {
                    case Axis.X:
                        mapping.jointTransform.localRotation = Quaternion.Euler(angle, 0, 0);
                        break;
                    case Axis.Y:
                        mapping.jointTransform.localRotation = Quaternion.Euler(0, angle, 0);
                        break;
                    case Axis.Z:
                        mapping.jointTransform.localRotation = Quaternion.Euler(0, 0, angle);
                        break;
                }
            }
        }
    }
    
    // Initialize LiDAR visualization components
    void InitializeLidarVisualization()
    {
        // Create a LineRenderer for LiDAR rays if not already present
        lidarLineRenderer = gameObject.AddComponent<LineRenderer>();
        lidarLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lidarLineRenderer.widthMultiplier = 0.02f;
        lidarLineRenderer.positionCount = 2; // Will be updated with actual data
        
        // Set up the line renderer with appropriate materials
        lidarLineRenderer.startColor = lidarColor;
        lidarLineRenderer.endColor = lidarColor;
        lidarLineRenderer.useWorldSpace = false;
        
        // Create individual point objects for visualization
        lidarPoints = new GameObject[maxLidarPoints];
        for (int i = 0; i < maxLidarPoints; i++)
        {
            GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point.name = "LidarPoint_" + i;
            point.transform.SetParent(transform);
            point.GetComponent<MeshRenderer>().material.color = lidarColor;
            
            // Make the sphere smaller
            point.transform.localScale = Vector3.one * 0.05f;
            point.SetActive(false);
            lidarPoints[i] = point;
        }
    }
    
    // Initialize depth texture for camera visualization
    void InitializeDepthTexture()
    {
        depthTexture = new Texture2D(640, 480); // Match camera resolution from config
        Graphics.Blit(Texture2D.blackTexture, depthTexture); // Initialize with black
    }
    
    // Update LiDAR visualization based on current readings
    void UpdateLidarVisualization()
    {
        if (lidarLineRenderer != null)
        {
            // Update line renderer based on LiDAR readings
            List<Vector3> positions = new List<Vector3>();
            
            for (int i = 0; i < lidarReadings.Length; i++)
            {
                float angle = Mathf.Deg2Rad * (i * 360.0f / lidarReadings.Length);
                float distance = lidarReadings[i];
                
                if (distance != Mathf.Infinity && distance > 0)
                {
                    float x = distance * Mathf.Cos(angle);
                    float z = distance * Mathf.Sin(angle);
                    
                    positions.Add(new Vector3(x, 0.1f, z)); // Slightly above ground level
                }
            }
            
            if (positions.Count > 0)
            {
                lidarLineRenderer.positionCount = positions.Count;
                lidarLineRenderer.SetPositions(positions.ToArray());
            }
        }
        
        // Update individual point visualization
        for (int i = 0; i < maxLidarPoints; i++)
        {
            if (i < lidarReadings.Length && lidarReadings[i] != Mathf.Infinity)
            {
                float angle = Mathf.Deg2Rad * (i * 360.0f / maxLidarPoints);
                float distance = lidarReadings[i];
                
                float x = distance * Mathf.Cos(angle);
                float z = distance * Mathf.Sin(angle);
                
                Vector3 pointPos = new Vector3(x, 0.1f, z);
                
                lidarPoints[i].transform.localPosition = pointPos;
                lidarPoints[i].SetActive(true);
            }
            else
            {
                if (i < lidarPoints.Length)
                    lidarPoints[i].SetActive(false);
            }
        }
    }
    
    // Update IMU visualization
    void UpdateImuVisualization()
    {
        // This could visualize the IMU orientation on the robot model
        // For example, by rotating an indicator object
    }
    
    // Update sensor visualizations
    void UpdateSensorVisualizations()
    {
        // Update LiDAR visualization
        UpdateLidarVisualization();
        
        // Update IMU indicator
        UpdateImuIndicator();
    }
    
    // Update IMU indicator visualization
    void UpdateImuIndicator()
    {
        // Update an IMU indicator object with the current orientation
    }
    
    // Synchronize with simulation state
    void SynchronizeWithSimulation()
    {
        // This method handles synchronization with the Gazebo simulation
        // In a real implementation, this would ensure Unity visualization 
        // stays in sync with the physics simulation
        float currentTime = Time.time;
        float timeDiff = Mathf.Abs(currentTime - lastSyncTime - (1.0f/syncFrequency));
        
        if (timeDiff > maxDesyncThreshold)
        {
            Debug.LogWarning($"Simulation desync detected: {timeDiff:F3}s (threshold: {maxDesyncThreshold}s)");
        }
        
        lastSyncTime = currentTime;
    }
    
    // Update robot movement toward target
    void UpdateRobotMovement()
    {
        if (!hasNewTarget) return;
        
        // Move toward target position
        robotRoot.position = Vector3.MoveTowards(robotRoot.position, targetPosition, moveSpeed * Time.deltaTime);
        
        // Rotate toward target rotation
        robotRoot.rotation = Quaternion.RotateTowards(robotRoot.rotation, targetRotation, turnSpeed * Mathf.Deg2Rad * Time.deltaTime);
        
        // Check if we've reached the target
        if (Vector3.Distance(robotRoot.position, targetPosition) < 0.01f)
        {
            hasNewTarget = false;
        }
    }
    
    // Public method to set robot target position
    public void SetRobotTarget(Vector3 position, Quaternion rotation)
    {
        targetPosition = position;
        targetRotation = rotation;
        hasNewTarget = true;
    }
    
    // Method to send joint commands to the robot
    public void SendJointCommands(Dictionary<string, float> commands)
    {
        JointStateMsg jointCommand = new JointStateMsg();
        jointCommand.name = new List<string>();
        jointCommand.position = new List<double>();
        
        foreach (var kvp in commands)
        {
            jointCommand.name.Add(kvp.Key);
            jointCommand.position.Add(kvp.Value);
        }
        
        ros.Publish(jointCommand, "/joint_commands");
    }
    
    // Method to send velocity commands
    public void SendVelocityCommand(float linearX, float angularZ)
    {
        TwistMsg velCmd = new TwistMsg();
        velCmd.linear = new Vector3Msg(linearX, 0, 0);  // Only x-direction for now
        velCmd.angular = new Vector3Msg(0, 0, angularZ);  // Only z-rotation
        ros.Publish(velCmd, "/cmd_vel");
    }
    
    // Method to get current joint positions
    public Dictionary<string, float> GetCurrentJointPositions()
    {
        return new Dictionary<string, float>(currentJointPositions);
    }
    
    // Method to get robot's current pose in the Unity coordinate system
    public (Vector3 position, Quaternion rotation) GetRobotPose()
    {
        if (robotRoot != null)
        {
            return (robotRoot.position, robotRoot.rotation);
        }
        else
        {
            return (transform.position, transform.rotation);
        }
    }
    
    // Method to apply external forces to the robot (for physics simulation)
    public void ApplyForceAtPosition(Vector3 force, Vector3 position, ForceMode mode = ForceMode.Force)
    {
        // In simulation, this would be handled by the physics engine
        // This is a placeholder for any Unity-side physics application
    }
}

// Helper class for joint mapping
[System.Serializable]
public class JointMapping
{
    public string jointName;
    public Transform jointTransform;
    public Axis rotationAxis;
}

// Enum for rotation axes
public enum Axis
{
    X, Y, Z
}