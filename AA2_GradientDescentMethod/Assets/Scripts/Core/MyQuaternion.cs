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

    //multiplication of two myquaternions (combining rotations)
    public static MyQuaternion operator *(MyQuaternion p_a, MyQuaternion p_b)
    {
        return new(
            p_a.w * p_b.x + p_a.x * p_b.w + p_a.y * p_b.z - p_a.z * p_b.y,
            p_a.w * p_b.y - p_a.x * p_b.z + p_a.y * p_b.w + p_a.z * p_b.x,
            p_a.w * p_b.z + p_a.x * p_b.y - p_a.y * p_b.x + p_a.z * p_b.w,
            p_a.w * p_b.w - p_a.x * p_b.x - p_a.y * p_b.y - p_a.z * p_b.z
        );
    }

    //create a quaternion from an angle and axis
    public static MyQuaternion AngleAxis(float p_angle, Vector3 p_axis)
    {
        if (p_axis.sqrMagnitude == 0.0f) { return identity; }

        float rad = p_angle * Mathf.Deg2Rad * 0.5f;
        float sin = Mathf.Sin(rad);
        float cos = Mathf.Cos(rad);

        Vector3 naxis = p_axis.normalized;

        return new MyQuaternion(naxis.x * sin, naxis.y * sin, naxis.z * sin, cos);
    }

    // Implicit conversions between MyQuaternion and Unity's Quaternion
    public static implicit operator Quaternion(MyQuaternion q) => new(q.x, q.y, q.z, q.w);
    public static implicit operator MyQuaternion(Quaternion q) => new(q.x, q.y, q.z, q.w);
}