using UnityEngine;

public class MyQuaternion
{
    public float x, y, z, w;

    //Quaternion constructors
    public MyQuaternion() { x = y = z = 0f; w = 1f; }
    public MyQuaternion(float p_x, float p_y, float p_z, float p_w)
    {
        x = p_x;
        y = p_y;
        z = p_z;
        w = p_w;
    }
    public MyQuaternion(MyVector3 p_v, float p_w)
    {
        x = p_v.x;
        y = p_v.y;
        z = p_v.z;
        w = p_w;
    }

    // Conjugate and inverse (just in case)
    public MyQuaternion Conjugate() => new(-x, -y, -z, w);
    public float SqrMagnitude() => x * x + y * y + z * z + w * w;
    public MyQuaternion Inverse()
    {
        float sq = SqrMagnitude();
        if (sq > 1e-6f)
        {
            MyQuaternion c = Conjugate();
            float inv = 1f / sq;
            return new(c.x * inv, c.y * inv, c.z * inv, c.w * inv);
        }
        return identity;
    }

    //Quaeternion identity
    public static MyQuaternion identity => new(0f, 0f, 0f, 1f);

    //Multiplication of two myquaternions (combining rotations)
    public static MyQuaternion operator *(MyQuaternion p_a, MyQuaternion p_b)
    {
        return new(
            p_a.w * p_b.x + p_a.x * p_b.w + p_a.y * p_b.z - p_a.z * p_b.y,
            p_a.w * p_b.y - p_a.x * p_b.z + p_a.y * p_b.w + p_a.z * p_b.x,
            p_a.w * p_b.z + p_a.x * p_b.y - p_a.y * p_b.x + p_a.z * p_b.w,
            p_a.w * p_b.w - p_a.x * p_b.x - p_a.y * p_b.y - p_a.z * p_b.z
        );
    }

    //Create a quaternion from an angle and axis
    public static MyQuaternion AngleAxis(float p_angle, Vector3 p_axis)
    {
        if (p_axis.sqrMagnitude == 0.0f) { return identity; }

        float rad = p_angle * Mathf.Deg2Rad * 0.5f;
        float sin = Mathf.Sin(rad);
        float cos = Mathf.Cos(rad);

        Vector3 naxis = p_axis.normalized;

        return new MyQuaternion(naxis.x * sin, naxis.y * sin, naxis.z * sin, cos);
    }

    // Slerp (Spherical Linear Interpolation) between two quaternions
    public static MyQuaternion Slerp(MyQuaternion p_a, MyQuaternion p_b, float p_t)
    {
        // Clamp t to [0, 1]
        p_t = Mathf.Clamp01(p_t);

        // Compute dot product
        float dot = p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z + p_a.w * p_b.w;

        // If dot < 0, negate one quaternion to take shorter path
        MyQuaternion b = p_b;
        if (dot < 0f)
        {
            b = new MyQuaternion(-p_b.x, -p_b.y, -p_b.z, -p_b.w);
            dot = -dot;
        }

        // If quaternions are very close, use linear interpolation
        if (dot > 0.9995f)
        {
            return new MyQuaternion(
                p_a.x + (b.x - p_a.x) * p_t,
                p_a.y + (b.y - p_a.y) * p_t,
                p_a.z + (b.z - p_a.z) * p_t,
                p_a.w + (b.w - p_a.w) * p_t
            );
        }

        // Calculate angle between quaternions
        float theta = Mathf.Acos(dot);
        float sinTheta = Mathf.Sin(theta);

        float wa = Mathf.Sin((1f - p_t) * theta) / sinTheta;
        float wb = Mathf.Sin(p_t * theta) / sinTheta;

        return new MyQuaternion(
            p_a.x * wa + b.x * wb,
            p_a.y * wa + b.y * wb,
            p_a.z * wa + b.z * wb,
            p_a.w * wa + b.w * wb
        );
    }

    // LookRotation - creates a rotation that looks along forward with upwards as up direction
    public static MyQuaternion LookRotation(MyVector3 p_forward, MyVector3 p_up)
    {
        // Normalize forward
        MyVector3 forward = p_forward.normalized;
        
        // Calculate right vector (cross product of up and forward)
        MyVector3 right = MyVector3.Cross(p_up, forward).normalized;
        
        // Recalculate up vector (cross product of forward and right)
        MyVector3 up = MyVector3.Cross(forward, right);

        // Build rotation matrix and convert to quaternion
        float m00 = right.x, m01 = right.y, m02 = right.z;
        float m10 = up.x, m11 = up.y, m12 = up.z;
        float m20 = forward.x, m21 = forward.y, m22 = forward.z;

        float trace = m00 + m11 + m22;
        MyQuaternion q = new MyQuaternion();

        if (trace > 0f)
        {
            float s = Mathf.Sqrt(trace + 1f) * 2f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        }
        else if (m00 > m11 && m00 > m22)
        {
            float s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            float s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        }
        else
        {
            float s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }

        return q;
    }

    // Implicit conversions between MyQuaternion and Unity's Quaternion
    public static implicit operator Quaternion(MyQuaternion q) => new(q.x, q.y, q.z, q.w);
    public static implicit operator MyQuaternion(Quaternion q) => new(q.x, q.y, q.z, q.w);
}