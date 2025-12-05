using UnityEngine;
using System.Collections.Generic;

public class OctopusArm : MonoBehaviour
{
    [Header("Arm Configuration")]
    [SerializeField] private int numSegments = 5;
    [SerializeField] private float segmentLength = 1.5f;
    [SerializeField] private GameObject segmentPrefab;
    
    [Header("IK Parameters")]
    [SerializeField] private float learningRate = 0.1f;
    [SerializeField] private int maxIterations = 50;
    [SerializeField] private float tolerance = 0.01f;
    
    [Header("Movement Constraints")]
    [SerializeField] private float maxAngleChange = 90f;
    [SerializeField] private float smoothSpeed = 5f;
    
    [Header("Grabbing")]
    [SerializeField] private float grabRadius = 1f;
    [SerializeField] private Transform grabPoint;
    
    // Arm structure
    private List<Transform> segments = new List<Transform>();
    private List<float> anglesX = new List<float>(); // Pitch
    private List<float> anglesY = new List<float>(); // Yaw
    private List<float> targetAnglesX = new List<float>();
    private List<float> targetAnglesY = new List<float>();
    
    private Transform target;
    private bool isGrabbing = false;
    private Transform grabbedTarget;

    void Start()
    {
        InitializeArm();
    }

    void InitializeArm()
    {
        // Create arm segments
        Transform parent = transform;
        
        for (int i = 0; i < numSegments; i++)
        {
            GameObject segment = Instantiate(segmentPrefab, parent.position, Quaternion.identity, parent);
            segment.transform.localPosition = new Vector3(0, 0, segmentLength);
            segments.Add(segment.transform);
            
            anglesX.Add(0f);
            anglesY.Add(0f);
            targetAnglesX.Add(0f);
            targetAnglesY.Add(0f);
            
            parent = segment.transform;
        }
        
        // Set grab point at the end
        if (grabPoint == null)
        {
            GameObject gp = new GameObject("GrabPoint");
            grabPoint = gp.transform;
            grabPoint.SetParent(segments[segments.Count - 1]);
            grabPoint.localPosition = new Vector3(0, 0, segmentLength);
        }
    }

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
    }

    void Update()
    {
        if (target != null && !isGrabbing)
        {
            // Solve IK using Gradient Descent
            SolveIK(target.position);
            
            // Smooth angle transitions
            SmoothAngles();
            
            // Apply rotations
            ApplyRotations();
            
            // Check if we can grab the target
            CheckGrab();
        }
        else if (isGrabbing)
        {
            // Keep target attached
            if (grabbedTarget != null)
            {
                grabbedTarget.position = grabPoint.position;
            }
        }
    }

    void SolveIK(Vector3 targetPosition)
    {
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // Calculate current end effector position
            MyVector3 endPos = CalculateEndEffectorPosition();
            MyVector3 targetPos = targetPosition;
            
            // Calculate error
            MyVector3 error = targetPos - endPos;
            float errorMagnitude = Mathf.Sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
            
            // Check if we're close enough
            if (errorMagnitude < tolerance)
                break;
            
            // Gradient descent for each joint
            for (int i = 0; i < segments.Count; i++)
            {
                // Calculate gradient for X rotation (pitch)
                float gradX = CalculateGradientX(i, targetPos);
                targetAnglesX[i] -= learningRate * gradX;
                
                // Calculate gradient for Y rotation (yaw)
                float gradY = CalculateGradientY(i, targetPos);
                targetAnglesY[i] -= learningRate * gradY;
                
                // Clamp angles to reasonable limits
                targetAnglesX[i] = Mathf.Clamp(targetAnglesX[i], -maxAngleChange, maxAngleChange);
                targetAnglesY[i] = Mathf.Clamp(targetAnglesY[i], -maxAngleChange, maxAngleChange);
            }
        }
    }

    float CalculateGradientX(int jointIndex, MyVector3 targetPos)
    {
        float delta = 0.01f;
        
        // Store original angle
        float originalAngle = targetAnglesX[jointIndex];
        
        // Calculate error with current angle
        MyVector3 pos1 = CalculateEndEffectorPosition();
        float error1 = CalculateError(pos1, targetPos);
        
        // Calculate error with perturbed angle
        targetAnglesX[jointIndex] += delta;
        MyVector3 pos2 = CalculateEndEffectorPosition();
        float error2 = CalculateError(pos2, targetPos);
        
        // Restore original angle
        targetAnglesX[jointIndex] = originalAngle;
        
        // Return gradient
        return (error2 - error1) / delta;
    }

    float CalculateGradientY(int jointIndex, MyVector3 targetPos)
    {
        float delta = 0.01f;
        
        float originalAngle = targetAnglesY[jointIndex];
        
        MyVector3 pos1 = CalculateEndEffectorPosition();
        float error1 = CalculateError(pos1, targetPos);
        
        targetAnglesY[jointIndex] += delta;
        MyVector3 pos2 = CalculateEndEffectorPosition();
        float error2 = CalculateError(pos2, targetPos);
        
        targetAnglesY[jointIndex] = originalAngle;
        
        return (error2 - error1) / delta;
    }

    float CalculateError(MyVector3 current, MyVector3 target)
    {
        MyVector3 diff = target - current;
        return diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    }

    MyVector3 CalculateEndEffectorPosition()
    {
        MyVector3 position = transform.position;
        MyQuaternion rotation = MyQuaternion.identity;
        
        for (int i = 0; i < segments.Count; i++)
        {
            // Apply rotations using custom quaternions
            MyQuaternion rotX = MyQuaternion.AngleAxis(targetAnglesX[i], Vector3.right);
            MyQuaternion rotY = MyQuaternion.AngleAxis(targetAnglesY[i], Vector3.up);
            rotation = rotation * rotY * rotX;
            
            // Calculate segment direction
            MyVector3 direction = rotation * new MyVector3(0, 0, segmentLength);
            position = position + direction;
        }
        
        return position;
    }

    void SmoothAngles()
    {
        for (int i = 0; i < segments.Count; i++)
        {
            anglesX[i] = Mathf.Lerp(anglesX[i], targetAnglesX[i], Time.deltaTime * smoothSpeed);
            anglesY[i] = Mathf.Lerp(anglesY[i], targetAnglesY[i], Time.deltaTime * smoothSpeed);
        }
    }

    void ApplyRotations()
    {
        for (int i = 0; i < segments.Count; i++)
        {
            MyQuaternion rotX = MyQuaternion.AngleAxis(anglesX[i], Vector3.right);
            MyQuaternion rotY = MyQuaternion.AngleAxis(anglesY[i], Vector3.up);
            MyQuaternion finalRot = rotY * rotX;
            
            segments[i].localRotation = finalRot;
        }
    }

    void CheckGrab()
    {
        if (target != null)
        {
            float distance = Vector3.Distance(grabPoint.position, target.position);
            
            if (distance < grabRadius && !isGrabbing)
            {
                Grab(target);
            }
        }
    }

    void Grab(Transform targetToGrab)
    {
        isGrabbing = true;
        grabbedTarget = targetToGrab;
        Debug.Log("Target grabbed!");
    }

    public void Release()
    {
        isGrabbing = false;
        grabbedTarget = null;
    }

    void OnDrawGizmos()
    {
        if (segments.Count > 0 && Application.isPlaying)
        {
            // Draw arm segments
            Gizmos.color = Color.red;
            for (int i = 0; i < segments.Count - 1; i++)
            {
                Gizmos.DrawLine(segments[i].position, segments[i + 1].position);
            }
            
            // Draw grab radius
            if (grabPoint != null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(grabPoint.position, grabRadius);
            }
        }
    }
}