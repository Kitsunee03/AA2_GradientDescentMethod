using UnityEngine;

public class IKGradient : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform Joint0;
    [SerializeField] private Transform Joint1;
    [SerializeField] private Transform Joint2;
    [SerializeField] private Transform endEffector;
    [SerializeField] private Transform target;

    [Header("Parameters")]
    [SerializeField] private float alpha = 0.1f;
    private float tolerance = 1f;
    private float costFunction;
    private Vector3 gradient;
    private Vector3 theta;

    private float l1, l2, l3;

    [Header("Adam Optimizer Parameters")]
    private float beta1 = 0.9f;
    private float beta2 = 0.999f;
    private float epsilon = 1e-8f;
    private int t = 1;
    private Vector3 m_t = Vector3.zero;
    private Vector3 v_t = Vector3.zero;

    [Header("Joint Limits (radians)")]
    [SerializeField] private Vector2 joint1Limits = new(-Mathf.PI * 0.25f, Mathf.PI * 0.25f); // -135 degrees to 135 degrees
    [SerializeField] private Vector2 joint2Limits = new(-Mathf.PI * 0.5f, Mathf.PI * 0.5f); // -90 degrees to 90 degrees
    [SerializeField] private Vector2 joint3Limits = new(-Mathf.PI * 0.5f, Mathf.PI * 0.5f); //-90 to 90

    private void Start()
    {
        l1 = Vector3.Distance(Joint0.position, Joint1.position);
        l2 = Vector3.Distance(Joint1.position, Joint2.position);
        l3 = Vector3.Distance(Joint2.position, endEffector.position);

        costFunction = Vector3.Distance(endEffector.position, target.position) * Vector3.Distance(endEffector.position, target.position);
        theta = Vector3.zero;
    }

    private void Update()
    {
        if (costFunction > tolerance)
        {
            gradient = CalculateGradient();

            Vector3 newAlpha = AdaptativeLearningRate(gradient); //adaptative alpha * gradient
            theta += -newAlpha;

            theta = ApplyingConstraints(theta); //angle constraints

            ForwardKinematics(theta); //update position
        }

        costFunction = Vector3.Distance(endEffector.position, target.position) * Vector3.Distance(endEffector.position, target.position);
    }


    private Vector3 ApplyingConstraints(Vector3 p_proposedAngles)
    {
        Vector3 constrainedAngles = p_proposedAngles;

        // theta0 - proposedAngles.x
        if (p_proposedAngles.x < joint1Limits.x) { constrainedAngles.x = joint1Limits.x; }
        else if (p_proposedAngles.x > joint1Limits.y) { constrainedAngles.x = joint1Limits.y; }

        //theta1 - proposedAngles.y
        if (p_proposedAngles.y < joint2Limits.x) { constrainedAngles.y = joint2Limits.x; }
        else if (p_proposedAngles.y > joint2Limits.y) { constrainedAngles.y = joint2Limits.y; }

        //theta2 - proposedAngles.z 
        if (p_proposedAngles.z < joint3Limits.x) { constrainedAngles.z = joint3Limits.x; }
        else if (p_proposedAngles.z > joint3Limits.y) { constrainedAngles.z = joint3Limits.y; }

        return constrainedAngles;
    }

    private  Vector3 AdaptativeLearningRate(Vector3 p_gradient)
    {
        t++;

        // update biased first and second moment estimates
        m_t = beta1 * m_t + (1 - beta1) * p_gradient;
        v_t = beta2 * v_t + (1 - beta2) * Vector3.Scale(p_gradient, p_gradient);

        // compute bias-corrected estimates
        Vector3 m_hat = m_t / (1 - Mathf.Pow(beta1, t));
        Vector3 v_hat = v_t / (1 - Mathf.Pow(beta2, t));

        // numerical stability: ensure the values inside sqrt/denominator are non-negative and not too small
        float vx = Mathf.Max(v_hat.x, epsilon);
        float vy = Mathf.Max(v_hat.y, epsilon);
        float vz = Mathf.Max(v_hat.z, epsilon);

        float denomX = Mathf.Sqrt(vx) + epsilon;
        float denomY = Mathf.Sqrt(vy) + epsilon;
        float denomZ = Mathf.Sqrt(vz) + epsilon;

        Vector3 adaptiveAlpha = new(
            alpha * m_hat.x / denomX,
            alpha * m_hat.y / denomY,
            alpha * m_hat.z / denomZ
        );

        // guard against NaN/Infinity from any numerical instability
        if (float.IsNaN(adaptiveAlpha.x) || float.IsInfinity(adaptiveAlpha.x)) { adaptiveAlpha.x = 0f; }
        if (float.IsNaN(adaptiveAlpha.y) || float.IsInfinity(adaptiveAlpha.y)) { adaptiveAlpha.y = 0f; }
        if (float.IsNaN(adaptiveAlpha.z) || float.IsInfinity(adaptiveAlpha.z)) { adaptiveAlpha.z = 0f; }

        return adaptiveAlpha;
    }


    private float Cost(Vector3 p_theta)
    {
        Vector3 endEffector = GetEndEffectorPosition(p_theta);
        return Vector3.Distance(endEffector, target.position) * Vector3.Distance(endEffector, target.position);
    }

    private Vector3 CalculateGradient()
    {
        Vector3 gradientVector;

        float step = 0.0001f;

        Vector3 thetaXPlus = new(theta.x + step, theta.y, theta.z);
        Vector3 thetaYPlus = new(theta.x, theta.y + step, theta.z);
        Vector3 thetaZPlus = new(theta.x, theta.y, theta.z + step);

        float DCostDX = (Cost(thetaXPlus) - Cost(theta)) / step;
        float DCostDY = (Cost(thetaYPlus) - Cost(theta)) / step;
        float DCostDZ = (Cost(thetaZPlus) - Cost(theta)) / step;


        gradientVector = new(DCostDX, DCostDY, DCostDZ);

        return gradientVector;
    }

    private Vector3 GetEndEffectorPosition(Vector3 p_theta)
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(p_theta.x)
                       + l2 * Mathf.Cos(p_theta.x + p_theta.y)
                       + l3 * Mathf.Cos(p_theta.x + p_theta.y + p_theta.z);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(p_theta.x)
                       + l2 * Mathf.Sin(p_theta.x + p_theta.y)
                       + l3 * Mathf.Sin(p_theta.x + p_theta.y + p_theta.z);

        newPosition.z = 0;

        return newPosition;
    }

    private Vector3 GetJoint2Position()
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x)
                       + l2 * Mathf.Cos(theta.x + theta.y);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x)
                       + l2 * Mathf.Sin(theta.x + theta.y);

        newPosition.z = 0;

        return newPosition;
    }

    private Vector3 GetJoint1Position()
    {
        Vector3 newPosition;

        newPosition.x = Joint0.position.x + l1 * Mathf.Cos(theta.x);
        newPosition.y = Joint0.position.y + l1 * Mathf.Sin(theta.x);

        newPosition.z = 0;

        return newPosition;
    }

    private void ForwardKinematics(Vector3 p_angle)
    {
        Vector3 eePos = GetEndEffectorPosition(p_angle);
        Vector3 j1Pos = GetJoint1Position();
        Vector3 j2Pos = GetJoint2Position();

        // validate before assigning to transforms
        bool eeValid = !(float.IsNaN(eePos.x) || float.IsNaN(eePos.y) || float.IsInfinity(eePos.x) || float.IsInfinity(eePos.y));
        bool j1Valid = !(float.IsNaN(j1Pos.x) || float.IsNaN(j1Pos.y) || float.IsInfinity(j1Pos.x) || float.IsInfinity(j1Pos.y));
        bool j2Valid = !(float.IsNaN(j2Pos.x) || float.IsNaN(j2Pos.y) || float.IsInfinity(j2Pos.x) || float.IsInfinity(j2Pos.y));

        if (eeValid) endEffector.position = eePos;
        else Debug.LogWarning("ForwardKinematics: computed NaN/Infinity for endEffector position — assignment skipped.", this);

        if (j1Valid) Joint1.position = j1Pos;
        else Debug.LogWarning("ForwardKinematics: computed NaN/Infinity for Joint1 position — assignment skipped.", this);

        if (j2Valid) Joint2.position = j2Pos;
        else Debug.LogWarning("ForwardKinematics: computed NaN/Infinity for Joint2 position — assignment skipped.", this);
    }
}