using UnityEngine;
using System.Collections.Generic;

public class IKGradientDynamic : MonoBehaviour
{
    [Header("Arm Joints")]
    [SerializeField] private List<Transform> joints;
    [SerializeField] private Transform target;

    [Header("IK Parameters")]
    [SerializeField] private float alpha = 0.1f;
    private const float tolerance = 0.001f;

    [Header("Joint Limits (radians)")]
    [SerializeField] private Vector2 jointLimits = new(-1.5f, 1.5f);

    // each joint has two DOF: yaw (y) and pitch (x)
    private float[] theta;
    private float[] lengths;
    private float costFunction;

    [Header("Adam optimizer")]
    private const float beta1 = 0.9f;
    private const float beta2 = 0.999f;
    private const float epsilon = 1e-8f;
    private float[] m, v;
    private int t = 0;

    private void Awake()
    {
        int n = joints.Count - 1;
        theta = new float[n * 2]; // 2 angles per joint (pitch, yaw)
        lengths = new float[n];

        for (int i = 0; i < n; i++)
            lengths[i] = Vector3.Distance(joints[i].position, joints[i + 1].position);

        m = new float[theta.Length];
        v = new float[theta.Length];

        costFunction = Cost(theta);
    }

    private void Update()
    {
        if (costFunction > tolerance)
        {
            float[] grad = CalculateGradient(theta);
            float[] step = AdamStep(grad);

            float[] newTheta = new float[theta.Length];
            for (int i = 0; i < theta.Length; i++)
                newTheta[i] = theta[i] - step[i];

            for (int i = 0; i < theta.Length; i++)
                newTheta[i] = Mathf.Clamp(newTheta[i], jointLimits.x, jointLimits.y);

            // normalize angles to [-pi, pi]
            for (int i = 0; i < newTheta.Length; i++)
                newTheta[i] = Mathf.Atan2(Mathf.Sin(newTheta[i]), Mathf.Cos(newTheta[i]));

            theta = newTheta;

            ForwardKinematics(theta);
        }

        costFunction = Cost(theta);
    }

    // ====================== IK Core =========================
    private float Cost(float[] p_theta)
    {
        Vector3 endEffector = ComputeEndEffector(p_theta);
        float distanceCost = Vector3.SqrMagnitude(endEffector - target.position);

        float angleCost = 0f;
        for (int i = 0; i < p_theta.Length; i++)
            angleCost += p_theta[i] * p_theta[i];

        return distanceCost + 0.001f * angleCost;
    }

    private float[] CalculateGradient(float[] p_theta)
    {
        float h = 0.0001f;
        float baseCost = Cost(p_theta);
        float[] grad = new float[p_theta.Length];
        float[] temp = new float[p_theta.Length];

        for (int i = 0; i < p_theta.Length; i++)
        {
            p_theta.CopyTo(temp, 0);
            temp[i] += h;
            grad[i] = (Cost(temp) - baseCost) / h;
        }

        return grad;
    }

    private float[] AdamStep(float[] p_grad)
    {
        t++;
        float[] step = new float[p_grad.Length];
        for (int i = 0; i < p_grad.Length; i++)
        {
            m[i] = beta1 * m[i] + (1 - beta1) * p_grad[i];
            v[i] = beta2 * v[i] + (1 - beta2) * (p_grad[i] * p_grad[i]);
            float m_hat = m[i] / (1 - Mathf.Pow(beta1, t));
            float v_hat = v[i] / (1 - Mathf.Pow(beta2, t));
            step[i] = alpha * m_hat / (Mathf.Sqrt(v_hat) + epsilon);
        }
        return step;
    }

    // ====================== FK Core =========================
    private Vector3 ComputeEndEffector(float[] p)
    {
        Vector3 pos = joints[0].position;
        Quaternion rotation = Quaternion.identity;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            float pitch = p[i * 2];
            float yaw = p[i * 2 + 1];
            Quaternion jointRotation = Quaternion.Euler(pitch * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg, 0f);
            rotation *= jointRotation;

            pos += rotation * Vector3.forward * lengths[i];
        }

        return pos;
    }

    private void ForwardKinematics(float[] p)
    {
        Vector3 pos = joints[0].position;
        Quaternion rotation = Quaternion.identity;

        for (int i = 0; i < joints.Count - 1; i++)
        {
            float pitch = p[i * 2];
            float yaw = p[i * 2 + 1];
            Quaternion jointRotation = Quaternion.Euler(pitch * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg, 0f);
            rotation *= jointRotation;

            Vector3 targetPos = pos + rotation * Vector3.forward * lengths[i];

            joints[i + 1].position = Vector3.Lerp(joints[i + 1].position, targetPos, 0.25f);
            pos = targetPos;
        }
    }
}