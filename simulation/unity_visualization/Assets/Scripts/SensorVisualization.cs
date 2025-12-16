using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ros2Sharp;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class SensorVisualization : MonoBehaviour
{
    [Header("LiDAR Visualization")]
    public GameObject lidarPointPrefab;  // Prefab for individual LiDAR points
    public LineRenderer lidarLineRenderer;  // Alternative visualization
    private List<GameObject> lidarPoints = new List<GameObject>();
    public int maxLidarPoints = 360;  // For 360-degree LiDAR
    
    [Header("IMU Visualization")]
    public GameObject imuIndicator;  // Visual indicator for IMU orientation
    public LineRenderer imuOrientationLine;  // Line indicating IMU orientation
    
    [Header("Depth Camera Visualization")]
    public Material depthMaterial;  // Material for depth visualization
    public Renderer depthCameraVisualizer;  // Visualizer for depth camera
    
    // Data received from sensors
    private float[] lidarReadings = new float[360];  // Simplified 360-point lidar
    private Vector3 imuOrientation = Vector3.zero;
    private Texture2D depthTexture;  // For visualizing depth camera data
    
    // Visualization settings
    public float lidarMaxDistance = 10.0f;  // Max distance in meters
    public Color lidarColor = Color.red;
    public Color imuColor = Color.blue;
    public float sensorUpdateInterval = 0.1f;  // Update every 100ms
    private float sensorUpdateTimer = 0f;
    
    // Robot reference
    public Transform robotTransform;
    
    // Initialize sensor visualization components
    void Start()
    {
        // Initialize LiDAR visualization
        InitializeLidarVisualization();
        
        // Initialize depth texture for camera visualization
        depthTexture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        
        // Set default values for lidar readings
        for (int i = 0; i < lidarReadings.Length; i++)
        {
            lidarReadings[i] = lidarMaxDistance;  // Default to max distance (no obstacle)
        }
        
        Debug.Log("SensorVisualization initialized");
    }
    
    // Update is called once per frame
    void Update()
    {
        // Periodically update sensor visualizations
        sensorUpdateTimer += Time.deltaTime;
        if (sensorUpdateTimer >= sensorUpdateInterval)
        {
            UpdateLidarVisualization();
            UpdateImuVisualization();
            UpdateDepthVisualization();
            sensorUpdateTimer = 0f;
        }
    }
    
    // Initialize LiDAR visualization components
    void InitializeLidarVisualization()
    {
        if (lidarLineRenderer != null)
        {
            lidarLineRenderer.positionCount = maxLidarPoints;
            lidarLineRenderer.startWidth = 0.02f;
            lidarLineRenderer.endWidth = 0.02f;
            lidarLineRenderer.startColor = lidarColor;
            lidarLineRenderer.endColor = lidarColor;
            lidarLineRenderer.useWorldSpace = false;
        }
        else
        {
            // Create points for visualization
            for (int i = 0; i < maxLidarPoints; i++)
            {
                if (lidarPointPrefab != null)
                {
                    GameObject pointObj = Instantiate(lidarPointPrefab, transform);
                    pointObj.SetActive(false);  // Initially inactive
                    lidarPoints.Add(pointObj);
                }
            }
        }
    }
    
    // Update LiDAR visualization based on received data
    void UpdateLidarVisualization()
    {
        if (lidarLineRenderer != null)
        {
            // Update line renderer positions based on lidar readings
            Vector3[] positions = new Vector3[lidarReadings.Length];
            for (int i = 0; i < lidarReadings.Length; i++)
            {
                float angle = Mathf.Deg2Rad * (i * 360.0f / lidarReadings.Length);
                float distance = Mathf.Clamp(lidarReadings[i], 0.0f, lidarMaxDistance);
                
                // Calculate position relative to robot
                float x = distance * Mathf.Cos(angle);
                float z = distance * Mathf.Sin(angle);
                
                positions[i] = new Vector3(x, 0.1f, z);  // Slightly above ground
            }
            
            lidarLineRenderer.SetPositions(positions);
        }
        else if (lidarPoints.Count > 0)
        {
            // Update individual point positions
            for (int i = 0; i < lidarReadings.Length && i < lidarPoints.Count; i++)
            {
                float angle = Mathf.Deg2Rad * (i * 360.0f / lidarReadings.Length);
                float distance = Mathf.Clamp(lidarReadings[i], 0.0f, lidarMaxDistance);
                
                float x = distance * Mathf.Cos(angle);
                float z = distance * Mathf.Sin(angle);
                
                Vector3 position = new Vector3(x, 0.1f, z);  // Relative to local coordinate system
                
                lidarPoints[i].transform.localPosition = position;
                lidarPoints[i].SetActive(distance < lidarMaxDistance - 0.1f);  // Activate if closer than max
            }
        }
    }
    
    // Update IMU visualization based on received data
    void UpdateImuVisualization()
    {
        if (imuIndicator != null)
        {
            // Update IMU indicator rotation to show orientation
            imuIndicator.transform.rotation = Quaternion.Euler(imuOrientation);
            
            // Change color based on orientation magnitude
            if (imuOrientation.magnitude > 0.1f)
            {
                imuIndicator.GetComponent<Renderer>().material.color = Color.red;  // Alert color for movement
            }
            else
            {
                imuIndicator.GetComponent<Renderer>().material.color = imuColor;   // Normal color
            }
        }
        
        if (imuOrientationLine != null)
        {
            // Draw a line showing the IMU's current orientation vector
            imuOrientationLine.SetPosition(0, transform.position);
            Vector3 orientationVector = new Vector3(imuOrientation.x, imuOrientation.y, imuOrientation.z);
            imuOrientationLine.SetPosition(1, transform.position + orientationVector * 0.5f);  // Scale for visibility
        }
    }
    
    // Update depth camera visualization
    void UpdateDepthVisualization()
    {
        // Update depth texture based on received depth data
        // This is a simplified example - real implementation would process depth map
        if (depthTexture != null)
        {
            // Create a simple visualization based on lidar readings
            // In a real system, this would be populated with actual depth camera data
            for (int x = 0; x < depthTexture.width; x++)
            {
                for (int y = 0; y < depthTexture.height; y++)
                {
                    // Map depth values to grayscale (simplified)
                    float normalizedDepth = 0.5f; // Default value
                    
                    // In real implementation: use actual depth data
                    // For demo, we'll map from lidar data
                    if (lidarReadings.Length > 0)
                    {
                        int lidarIdx = (int)(((float)x / depthTexture.width) * lidarReadings.Length);
                        if (lidarIdx >= 0 && lidarIdx < lidarReadings.Length)
                        {
                            normalizedDepth = lidarReadings[lidarIdx] / lidarMaxDistance;
                        }
                    }
                    
                    // Map to grayscale color
                    Color depthColor = Color.gray * normalizedDepth;
                    depthTexture.SetPixel(x, y, depthColor);
                }
            }
            
            depthTexture.Apply();
            
            // Apply texture to visualizer if available
            if (depthCameraVisualizer != null && depthCameraVisualizer.material != null)
            {
                depthCameraVisualizer.material.mainTexture = depthTexture;
            }
        }
    }
    
    // Method to receive and update LiDAR data from ROS
    public void UpdateLidarData(float[] ranges)
    {
        // Make sure the array sizes match
        if (lidarReadings.Length == ranges.Length)
        {
            for (int i = 0; i < lidarReadings.Length; i++)
            {
                lidarReadings[i] = (float)ranges[i];
            }
        }
        else
        {
            // Handle mismatched array sizes - copy what we can
            int copySize = Mathf.Min(lidarReadings.Length, ranges.Length);
            for (int i = 0; i < copySize; i++)
            {
                lidarReadings[i] = (float)ranges[i];
            }
        }
    }
    
    // Method to receive and update IMU data from ROS
    public void UpdateImuData(float x, float y, float z)
    {
        imuOrientation = new Vector3(x, y, z);
    }
    
    // Method to receive and update depth camera data from ROS
    public void UpdateDepthCameraData(float[,] depthMap)
    {
        // Update the depth texture with new data
        // In a real implementation, this would process the actual depth camera frame
        if (depthMap != null)
        {
            for (int x = 0; x < Mathf.Min(depthTexture.width, depthMap.GetLength(0)); x++)
            {
                for (int y = 0; y < Mathf.Min(depthTexture.height, depthMap.GetLength(1)); y++)
                {
                    float depthValue = depthMap[x, y];
                    float normalizedDepth = depthValue / lidarMaxDistance; // Simplified normalization
                    Color depthColor = Color.gray * normalizedDepth;
                    depthTexture.SetPixel(x, y, depthColor);
                }
            }
            
            depthTexture.Apply();
        }
    }
    
    // Method to receive and update depth camera data from ROS (alternative format)
    public void UpdateDepthCameraData(byte[] imageData, int width, int height)
    {
        // Update the depth texture with image data in byte array format
        if (imageData != null && imageData.Length == width * height * 4) // RGBA
        {
            for (int x = 0; x < Mathf.Min(width, depthTexture.width); x++)
            {
                for (int y = 0; y < Mathf.Min(height, depthTexture.height); y++)
                {
                    int pixelIndex = (y * width + x) * 4; // RGBA format
                    float r = imageData[pixelIndex] / 255.0f;
                    float g = imageData[pixelIndex + 1] / 255.0f;
                    float b = imageData[pixelIndex + 2] / 255.0f;
                    
                    // Convert RGB to grayscale for depth visualization
                    float gray = (r + g + b) / 3.0f;
                    Color depthColor = Color.gray * gray;
                    depthTexture.SetPixel(x, y, depthColor);
                }
            }
            
            depthTexture.Apply();
        }
    }
    
    // Method to get sensor status for UI/debugging
    public string GetSensorStatusString()
    {
        System.Text.StringBuilder sb = new System.Text.StringBuilder();
        sb.AppendLine("Sensor Status:");
        
        // LiDAR stats
        float minRange = Mathf.Infinity, maxRange = 0f;
        foreach (float range in lidarReadings)
        {
            if (range < minRange) minRange = range;
            if (range > maxRange) maxRange = range;
        }
        sb.AppendLine($"LiDAR: Min={minRange:F2}m, Max={maxRange:F2}m");
        
        // IMU status
        sb.AppendLine($"IMU: Ori=({imuOrientation.x:F2}, {imuOrientation.y:F2}, {imuOrientation.z:F2})");
        
        return sb.ToString();
    }
    
    // Cleanup resources
    void OnDestroy()
    {
        if (depthTexture != null)
        {
            Destroy(depthTexture);
        }
        
        // Destroy created visualization points
        foreach (GameObject point in lidarPoints)
        {
            if (point != null)
            {
                DestroyImmediate(point);
            }
        }
    }
}