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

        // movimiento deseado en world space (MyVector3 -> Vector3)
        Vector3 desiredMovement = (Vector3)randomDirection * moveSpeed * Time.deltaTime;
        Vector3 desiredPos = transform.position + desiredMovement;

        // obtener posición final limitada por el collider (ClosestPoint)
        if (planeBounds != null)
        {
            Vector3 surfacePoint = planeBounds.ClosestPoint(desiredPos);
            desiredPos = new(surfacePoint.x, surfacePoint.y, surfacePoint.z);
        }
        else
        {
            desiredPos.x = Mathf.Clamp(desiredPos.x, movementAreaMin.x, movementAreaMax.x);
            desiredPos.z = Mathf.Clamp(desiredPos.z, movementAreaMin.y, movementAreaMax.y);
            desiredPos.y = Mathf.Max(desiredPos.y, 0.5f);
        }

        // guardar la dirección real del movimiento antes de mover (para rotación)
        Vector3 moveDir = desiredPos - transform.position;
        // proyectar en XZ para evitar tilts por diferencias en Y
        moveDir.y = 0f;

        // mover respetando Rigidbody
        if (rb != null) { rb.MovePosition(desiredPos); }
        else { transform.position = desiredPos; }

        // rotar hacia la dirección real del movimiento usando LookRotation + offset
        if (moveDir.sqrMagnitude > 1e-6f)
        {
            Quaternion targetRot = Quaternion.LookRotation(moveDir.normalized, Vector3.up)
                                   * Quaternion.Euler(0f, 0f, 0f);
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