using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class ROSConnector : MonoBehaviour
{
    private ROSConnection ros;
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;

    // Robot joint control
    public GameObject humanoidRobot;

    // Sensor data
    public TextMesh sensorDataText; // For displaying sensor data in the scene

    // Simulation state
    private float[] jointPositions = new float[20]; // Assuming 20 DOF for humanoid
    private float[] jointVelocities = new float[20];
    private float[] jointEfforts = new float[20];

    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection static instance, which will remain active between scenes
        ros = ROSConnection.GetOrCreateInstance();
        
        // Set the IP and port to connect to
        ros.Initialize(rosIP, rosPort);

        // Subscribe to robot state topic
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Nav_msgs.OdometryMsg>("robot_odom", OnRobotOdometryReceived);

        // Subscribe to joint states topic
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.JointStateMsg>("joint_states", OnJointStatesReceived);

        // Subscribe to sensor data topics
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.LaserScanMsg>("sensors/lidar/scan", OnLidarDataReceived);
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.ImuMsg>("sensors/imu/data", OnImuDataReceived);

        Debug.Log($"ROSConnector initialized with IP: {rosIP}, Port: {rosPort}");
    }

    // Callback for receiving robot odometry data
    void OnRobotOdometryReceived(Unity.Robotics.ROSTCPConnector.MessageTypes.Nav_msgs.OdometryMsg odomMsg)
    {
        if (humanoidRobot != null)
        {
            // Update position and orientation based on odometry
            Vector3 position = new Vector3((float)odomMsg.pose.pose.position.x, 
                                         (float)odomMsg.pose.pose.position.y, 
                                         (float)odomMsg.pose.pose.position.z);
            Quaternion rotation = new Quaternion((float)odomMsg.pose.pose.orientation.x,
                                               (float)odomMsg.pose.pose.orientation.y,
                                               (float)odomMsg.pose.pose.orientation.z,
                                               (float)odomMsg.pose.pose.orientation.w);

            humanoidRobot.transform.position = position;
            humanoidRobot.transform.rotation = rotation;
        }

        Debug.Log($"Received odometry: pos=({position.x}, {position.y}, {position.z})");
    }

    // Callback for receiving joint states
    void OnJointStatesReceived(JointStateMsg jointStateMsg)
    {
        // Update joint positions in the humanoid model
        // This would require accessing the robot's joints and updating their angles
        if (jointStateMsg.position.Length > 0)
        {
            // Copy positions to local array
            for (int i = 0; i < Mathf.Min(jointPositions.Length, jointStateMsg.position.Length); i++)
            {
                jointPositions[i] = (float)jointStateMsg.position[i];
            }

            // Update joint velocities and efforts too
            for (int i = 0; i < Mathf.Min(jointVelocities.Length, jointStateMsg.velocity.Length); i++)
            {
                jointVelocities[i] = (float)jointStateMsg.velocity[i];
            }

            for (int i = 0; i < Mathf.Min(jointEfforts.Length, jointStateMsg.effort.Length); i++)
            {
                jointEfforts[i] = (float)jointStateMsg.effort[i];
            }

            // Update the robot's joints (implementation depends on how joints are set up)
            UpdateHumanoidJoints();

            if (sensorDataText != null)
            {
                sensorDataText.text = $"Joints: {jointStateMsg.position.Length}\n" +
                                    $"First Pos: {(float)jointStateMsg.position[0]:F2}";
            }
        }
    }

    // Callback for receiving LiDAR data
    void OnLidarDataReceived(LaserScanMsg lidarMsg)
    {
        Debug.Log($"Received LiDAR scan with {lidarMsg.ranges.Length} points");

        // Display or process LiDAR data here
        // For visualization, you might want to create point clouds or other visual elements
    }

    // Callback for receiving IMU data
    void OnImuDataReceived(ImuMsg imuMsg)
    {
        Vector3 orientation = new Vector3((float)imuMsg.orientation.x,
                                        (float)imuMsg.orientation.y,
                                        (float)imuMsg.orientation.z);
        // Process IMU data
        Debug.Log($"Received IMU data: orientation=({orientation.x}, {orientation.y}, {orientation.z})");
    }

    // Update the humanoid robot's joints based on received positions
    void UpdateHumanoidJoints()
    {
        // This is a stub - in a real implementation, you would need to access
        // the specific joints of your humanoid model and update their rotations

        // Example: Update some joints (assuming you have references to joint objects)
        // joint1.transform.localRotation = Quaternion.Euler(0, jointPositions[0] * Mathf.Rad2Deg, 0);
        // joint2.transform.localRotation = Quaternion.Euler(0, jointPositions[1] * Mathf.Rad2Deg, 0);
        // ... continue for other joints

        // For a more sophisticated implementation, you would likely use a kinematic chain
        // or inverse kinematics system to update the humanoid model appropriately
    }

    // Method to send joint commands (if needed)
    public void SendJointCommand(string[] jointNames, double[] positions, double[] velocities, double[] efforts)
    {
        var jointCmd = new JointStateMsg();
        jointCmd.name = jointNames;
        jointCmd.position = positions;
        jointCmd.velocity = velocities;
        jointCmd.effort = efforts;
        jointCmd.header = new HeaderMsg();
        jointCmd.header.stamp = new TimeMsg();
        jointCmd.header.frame_id = "base_link";

        ros.Send("joint_commands", jointCmd);
    }

    // Update is called once per frame
    void Update()
    {
        // You can send data to ROS here if needed
        // For example, sending the robot's current position back to ROS
    }

    // OnDestroy is called when the object is destroyed
    void OnDestroy()
    {
        if (ros != null)
        {
            ros.Close();
        }
    }
}