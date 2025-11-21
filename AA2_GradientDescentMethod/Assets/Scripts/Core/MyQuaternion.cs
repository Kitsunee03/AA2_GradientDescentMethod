using UnityEngine;

public class MyQuaternion 
{

    public float x, y, z, w;

    //Quaternion constructors
    public MyQuaternion() {x = y = z = 0f; w = 1f; }
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
    public MyQuaternion Conjugate() => new MyQuaternion(-x, -y, -z, w);
    public MyQuaternion Inverse()
    {
        float sq = SqrMagnitude();
        if (sq > 1e-6f)
        {
            MyQuaternion c = Conjugate();
            float inv = 1f / sq;
            return new MyQuaternion(c.x * inv, c.y * inv, c.z * inv, c.w * inv);
        }
        return identity;
    }

    //Quaeternion identity
    public static MyQuaternion identity => new MyQuaternion(0f, 0f, 0f, 1f);

    //multiplication of two myquaternions (combining rotations)
    public static MyQuaternion operator *(MyQuaternion a, MyQuaternion b)
    {
        return new MyQuaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        );
    }

    //create a quaternion from an angle and axis
    public static MyQuaternion AngleAxis(float angle, Vector3 axis)
    {
        if (axis.sqrMagnitude == 0.0f) return MyQuaternion.identity;

        float rad = angle * Mathf.Deg2Rad * 0.5f;
        float sin = Mathf.Sin(rad);
        float cos = Mathf.Cos(rad);

        Vector3 naxis = axis.normalized;

        return new MyQuaternion(naxis.x * sin, naxis.y * sin, naxis.z * sin, cos);
    }

     // Implicit conversions between MyQuaternion and Unity's Quaternion
    public static implicit operator Quaternion(MyQuaternion q) => new Quaternion(q.x, q.y, q.z, q.w);
    public static implicit operator MyQuaternion(Quaternion q) => new MyQuaternion(q.x, q.y, q.z, q.w);
}
