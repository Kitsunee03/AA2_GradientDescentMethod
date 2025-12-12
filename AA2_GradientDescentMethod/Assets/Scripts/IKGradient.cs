using UnityEngine;
using System.Collections.Generic;

public class IKGradient : MonoBehaviour
{
    private bool isInUse = false;

    [Header("Joint Configuration")]
    [SerializeField] private Transform rootJoint;
    [SerializeField] private List<Transform> joints = new();
    [SerializeField] private Collider targetCollider;
    [SerializeField] private Transform idleTarget;
    private MyVector3 targetPoint;

    [Header("Optimization Parameters")]
    [SerializeField] private float alpha = 0.08f;
    [SerializeField] private float tolerance = 0.01f;
    [SerializeField] private float smoothingFactor = 0.45f;
    [SerializeField] private float gradientDamping = 1.2f;

    // rotation limits
    private const float collisionPenalty = 2.0f;
    private const float minDistanceBetweenJoints = 0.3f;

    private float costFunction;
    private float[] angles; // [theta_x, theta_y, theta_z] per joint
    private float[] angleVelocities, previousGradient, linkLengths;
    private MyVector3[] linkDirections, initialPositions;
    private int totalDOF;

    private void Awake() { InitializeIK(); }

    private void InitializeIK()
    {
        if (joints.Count == 0)
        {
            Debug.LogError("No joints assigned!");
            return;
        }

        totalDOF = joints.Count * 3;

        angles = new float[totalDOF];
        angleVelocities = new float[totalDOF];
        previousGradient = new float[totalDOF];
        linkLengths = new float[joints.Count];
        linkDirections = new MyVector3[joints.Count];
        initialPositions = new MyVector3[joints.Count];

        for (int i = 0; i < joints.Count; i++) { initialPositions[i] = joints[i].position; }
        for (int i = 0; i < joints.Count; i++)
        {
            MyVector3 startPos = (i == 0) ? (MyVector3)rootJoint.position : initialPositions[i - 1];
            MyVector3 endPos = initialPositions[i];

            MyVector3 linkVector = endPos - startPos;
            linkLengths[i] = Mathf.Sqrt(linkVector.x * linkVector.x + linkVector.y * linkVector.y + linkVector.z * linkVector.z);
            float sqrMag = linkVector.x * linkVector.x + linkVector.y * linkVector.y + linkVector.z * linkVector.z;
            linkDirections[i] = sqrMag > 0.00001f ? linkVector.normalized : MyVector3.right;
        }

        for (int i = 0; i < totalDOF; i++)
        {
            angles[i] = 0f;
            angleVelocities[i] = 0f;
            previousGradient[i] = 0f;
        }

        costFunction = CalculateCost(angles);
    }

    private void Update()
    {
        if (joints.Count == 0 || targetPoint == null) { return; }

        if (costFunction > tolerance)
        {
            float[] gradient = CalculateGradient();

            // apply gradient with smoothing and damping
            for (int i = 0; i < totalDOF; i++)
            {
                float deltaAngle = -alpha * gradient[i];
                angleVelocities[i] = angleVelocities[i] * (1f - smoothingFactor) + deltaAngle * smoothingFactor;
                angles[i] += angleVelocities[i] * gradientDamping;
            }

            ForwardKinematics();
        }

        costFunction = CalculateCost(angles);
    }

    // ------------------------
    // COST FUNCTION
    // ------------------------

    private float CalculateCost(float[] p_theta)
    {
        MyVector3 endEffectorPos = GetEndEffectorPosition(p_theta);

        // target point
        if (isInUse) { targetPoint = targetCollider.ClosestPoint(joints[0].position); }
        else { targetPoint = idleTarget.position; }

        MyVector3 diff = endEffectorPos - targetPoint;

        float distance = Mathf.Sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
        float positionCost = distance * distance;

        // penalize collisions between joints
        float collisionCost = CalculateCollisionPenalty(p_theta);

        return positionCost + collisionPenalty * collisionCost;
    }

    private float CalculateCollisionPenalty(float[] p_theta)
    {
        float penalty = 0f;
        MyVector3[] jointPositions = GetJointPositions(p_theta);

        float warningRadius = minDistanceBetweenJoints * 0.75f;
        float hardLimit = minDistanceBetweenJoints * 0.55f;

        for (int i = 0; i < jointPositions.Length; i++)
        {
            for (int j = i + 2; j < jointPositions.Length; j++)
            {
                MyVector3 diff = jointPositions[i] - jointPositions[j];
                float dist = Mathf.Sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);

                // safe zone: no penalty
                if (dist >= warningRadius) { continue; }

                //  intermediate zone: soft penalty
                if (dist >= hardLimit)
                {
                    float t = (warningRadius - dist) / (warningRadius - hardLimit);
                    float softPenalty = t * t * 0.15f;  // suave, no interfiere
                    penalty += softPenalty;
                    continue;
                }

                // critical zone: strong penalty
                float violation = Mathf.Max(0f, hardLimit - dist);
                penalty += violation * violation * 1.6f;
            }
        }

        // clamp maximum penalty to avoid extreme values
        return Mathf.Min(penalty, 4f);
    }

    // ------------------------
    // IK GRADIENT CALCULATION
    // ------------------------

    private MyVector3[] GetJointPositions(float[] p_theta)
    {
        MyVector3[] positions = new MyVector3[joints.Count];
        MyVector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;

            float angleX = p_theta[baseIdx] * Mathf.Rad2Deg;
            float angleY = p_theta[baseIdx + 1] * Mathf.Rad2Deg;
            float angleZ = p_theta[baseIdx + 2] * Mathf.Rad2Deg;

            MyQuaternion rotX = MyQuaternion.AngleAxis(angleX, Vector3.right);
            MyQuaternion rotY = MyQuaternion.AngleAxis(angleY, Vector3.up);
            MyQuaternion rotZ = MyQuaternion.AngleAxis(angleZ, Vector3.forward);
            MyQuaternion rotationDelta = rotZ * rotY * rotX;

            MyVector3 rotatedDirection = rotationDelta * linkDirections[i];

            currentPos = currentPos + rotatedDirection * linkLengths[i];
            positions[i] = currentPos;
        }

        return positions;
    }

    private float[] CalculateGradient()
    {
        float[] gradient = new float[totalDOF];
        float step = 0.001f;

        float baseCost = CalculateCost(angles);

        for (int i = 0; i < totalDOF; i++)
        {
            float[] thetaPlus = (float[])angles.Clone();
            thetaPlus[i] += step;

            float costPlus = CalculateCost(thetaPlus);
            gradient[i] = (costPlus - baseCost) / step;

            // clamp gradients to avoid abrupt changes
            gradient[i] = Mathf.Clamp(gradient[i], -1f, 1f);

            // smooth with history for stability
            gradient[i] = Mathf.Lerp(previousGradient[i], gradient[i], 0.6f);
            previousGradient[i] = gradient[i];
        }

        return gradient;
    }

    // ------------------------
    // FORWARD KINEMATICS
    // ------------------------

    private MyVector3 GetEndEffectorPosition(float[] p_theta)
    {
        MyVector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;

            float angleX = p_theta[baseIdx] * Mathf.Rad2Deg;
            float angleY = p_theta[baseIdx + 1] * Mathf.Rad2Deg;
            float angleZ = p_theta[baseIdx + 2] * Mathf.Rad2Deg;

            MyQuaternion rotX = MyQuaternion.AngleAxis(angleX, Vector3.right);
            MyQuaternion rotY = MyQuaternion.AngleAxis(angleY, Vector3.up);
            MyQuaternion rotZ = MyQuaternion.AngleAxis(angleZ, Vector3.forward);
            MyQuaternion rotationDelta = rotZ * rotY * rotX;

            MyVector3 rotatedDirection = rotationDelta * linkDirections[i];

            currentPos = currentPos + rotatedDirection * linkLengths[i];
        }

        return currentPos;
    }

    private void ForwardKinematics()
    {
        MyVector3 currentPos = rootJoint.position;

        for (int i = 0; i < joints.Count; i++)
        {
            int baseIdx = i * 3;

            float angleX = angles[baseIdx] * Mathf.Rad2Deg;
            float angleY = angles[baseIdx + 1] * Mathf.Rad2Deg;
            float angleZ = angles[baseIdx + 2] * Mathf.Rad2Deg;

            MyQuaternion rotX = MyQuaternion.AngleAxis(angleX, Vector3.right);
            MyQuaternion rotY = MyQuaternion.AngleAxis(angleY, Vector3.up);
            MyQuaternion rotZ = MyQuaternion.AngleAxis(angleZ, Vector3.forward);
            MyQuaternion rotationDelta = rotZ * rotY * rotX;

            MyVector3 rotatedDirection = rotationDelta * linkDirections[i];

            MyVector3 nextPos = currentPos + rotatedDirection * linkLengths[i];

            // smooth position and rotation - convert to Unity for transform operations
            Vector3 currentUnityPos = joints[i].position;
            Vector3 targetUnityPos = nextPos;
            joints[i].position = Vector3.Lerp(currentUnityPos, targetUnityPos, 0.2f);

            // rotation to look at previous joint
            MyVector3 targetLookPos = (i > 0) ? (MyVector3)joints[i - 1].position : (MyVector3)rootJoint.position;
            MyVector3 jointPos = joints[i].position;
            MyVector3 directionToPrev = (targetLookPos - jointPos).normalized;

            float sqrMag = directionToPrev.x * directionToPrev.x + directionToPrev.y * directionToPrev.y + directionToPrev.z * directionToPrev.z;
            if (sqrMag > 0.00001f)
            {
                Vector3 unityDir = directionToPrev;
                Quaternion targetRotation = Quaternion.LookRotation(unityDir, Vector3.right) * Quaternion.Euler(0, -90, 0);
                joints[i].rotation = Quaternion.Lerp(joints[i].rotation, targetRotation, 0.15f);
            }

            currentPos = nextPos;
        }
    }

    public float GetDistanceToTarget()
    {
        return MyVector3.Distance(joints[joints.Count - 1].position, targetCollider.ClosestPoint(joints[0].position));
    }

    public bool IsInUse { get { return isInUse; } set { isInUse = value; } }

    // ------------------------
    // GIZMOS
    // ------------------------

    private void OnDrawGizmos()
    {
        if (joints.Count == 0 || rootJoint == null) { return; }

        Gizmos.color = Color.yellow;
        Vector3 prevPos = rootJoint.position;

        foreach (Transform joint in joints)
        {
            if (joint != null)
            {
                Gizmos.DrawLine(prevPos, joint.position);
                Gizmos.DrawSphere(joint.position, 0.05f);
                prevPos = joint.position;
            }
        }

        // Draw last joint as end effector in green
        if (joints.Count > 0 && joints[joints.Count - 1] != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(joints[joints.Count - 1].position, 0.08f);
        }

        if (targetPoint != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(targetPoint, 0.5f);
        }
    }
}