using UnityEngine;

public class MyVector3
{
    public float x, y, z;

    public MyVector3(float p_x, float p_y, float p_z)
    {
        x = p_x;
        y = p_y;
        z = p_z;
    }

    public static MyVector3 right { get { return new(1, 0, 0); } }
    public static MyVector3 forward { get { return new(0, 0, 1); } }
    public static MyVector3 up { get { return new(0, 1, 0); } }
    public static MyVector3 zero { get { return new(0, 0, 0); } }

    // complex vector operations
    public MyVector3 normalized
    {
        get
        {
            float magnitude = Mathf.Sqrt(x * x + y * y + z * z);
            if (magnitude > 0) { return new(x / magnitude, y / magnitude, z / magnitude); }

            return new(0, 0, 0);
        }
    }

    // basic vector operations
    public static MyVector3 operator +(MyVector3 p_a, MyVector3 p_b)
    {
        return new(p_a.x + p_b.x, p_a.y + p_b.y, p_a.z + p_b.z);
    }

    public static MyVector3 operator -(MyVector3 p_a, MyVector3 p_b)
    {
        return new(p_a.x - p_b.x, p_a.y - p_b.y, p_a.z - p_b.z);
    }

    public static MyVector3 operator *(MyVector3 p_a, float p_scalar)
    {
        return new(p_a.x * p_scalar, p_a.y * p_scalar, p_a.z * p_scalar);
    }

    //multiplication of myvector3 with myquaternion
    public static MyVector3 operator *(MyQuaternion q, MyVector3 v)
    {
        // t = 2 * cross(q.xyz, v)
        float tx = 2f * (q.y * v.z - q.z * v.y);
        float ty = 2f * (q.z * v.x - q.x * v.z);
        float tz = 2f * (q.x * v.y - q.y * v.x);

        // v' = v + q.w * t + cross(q.xyz, t)
        float cx = q.y * tz - q.z * ty;
        float cy = q.z * tx - q.x * tz;
        float cz = q.x * ty - q.y * tx;

        return new MyVector3(
            v.x + q.w * tx + cx,
            v.y + q.w * ty + cy,
            v.z + q.w * tz + cz
        );
    }

    // implicit conversions between MyVector3 and Unity's Vector3
    public static implicit operator Vector3(MyVector3 p_v) { return new(p_v.x, p_v.y, p_v.z); }

    public static implicit operator MyVector3(Vector3 p_v) { return new(p_v.x, p_v.y, p_v.z); }
}