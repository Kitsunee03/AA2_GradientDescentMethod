using UnityEngine;

public class SpiderManController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;

    [Header("Plane Bounds")]
    [SerializeField] private Collider planeBounds;

    [Header("Random Movement")]
    [SerializeField] private float changeDirectionInterval = 2f;
    [SerializeField] private Vector2 movementAreaMin = new(-10, -10);
    [SerializeField] private Vector2 movementAreaMax = new(10, 10);

    private Rigidbody rb;
    private MyVector3 randomDirection;
    private float directionTimer;

    private void Start()
    {
        TryGetComponent(out rb);
        if (rb == null) { rb = gameObject.AddComponent<Rigidbody>(); }

        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeRotation;

        ChooseRandomDirection();
        directionTimer = changeDirectionInterval;
    }

    private void Update() { RandomMovement(); }

    private void RandomMovement()
    {
        directionTimer -= Time.deltaTime;

        if (directionTimer <= 0)
        {
            ChooseRandomDirection();
            directionTimer = changeDirectionInterval;
        }

        // movimiento deseado en world space usando MyVector3
        MyVector3 desiredMovement = randomDirection * moveSpeed * Time.deltaTime;
        MyVector3 desiredPos = (MyVector3)transform.position + desiredMovement;

        // obtener posición final limitada por el collider (ClosestPoint)
        if (planeBounds != null)
        {
            // ClosestPoint requiere Vector3, convertimos y volvemos a MyVector3
            Vector3 surfacePoint = planeBounds.ClosestPoint((Vector3)desiredPos);
            desiredPos = new MyVector3(surfacePoint.x, surfacePoint.y, surfacePoint.z);
        }
        else
        {
            desiredPos.x = Mathf.Clamp(desiredPos.x, movementAreaMin.x, movementAreaMax.x);
            desiredPos.z = Mathf.Clamp(desiredPos.z, movementAreaMin.y, movementAreaMax.y);
            desiredPos.y = Mathf.Max(desiredPos.y, 0.5f);
        }

        // guardar la dirección real del movimiento antes de mover (para rotación)
        MyVector3 moveDir = desiredPos - (MyVector3)transform.position;
        // proyectar en XZ para evitar tilts por diferencias en Y
        moveDir.y = 0f;

        // mover respetando Rigidbody
        if (rb != null) { rb.MovePosition((Vector3)desiredPos); }
        else { transform.position = (Vector3)desiredPos; }

        // rotar hacia la dirección real del movimiento usando LookRotation + offset
        // calcular magnitud cuadrada manualmente para MyVector3
        float moveDirSqrMag = moveDir.x * moveDir.x + moveDir.y * moveDir.y + moveDir.z * moveDir.z;
        if (moveDirSqrMag > 1e-6f)
        {
            Vector3 unityDir = (Vector3)moveDir.normalized;
            Quaternion targetRot = Quaternion.LookRotation(unityDir, Vector3.up);
            // suavizar rotación
            if (rb != null) { rb.MoveRotation(Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * 5f)); }
            else { transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * 5f); }
        }
    }

    void ChooseRandomDirection()
    {
        int direction = Random.Range(0, 3); // 0..2
        randomDirection = direction switch
        {
            0 => MyVector3.right,
            1 => MyVector3.left,
            2 => MyVector3.forward,
            _ => MyVector3.right
        };
    }
}