using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class self_driving_camera_manager : MonoBehaviour
{
    public float fov = 90f;
    private Camera[] cameras;

    // Start is called before the first frame update
    void Start()
    {
        // Get camera components from children
        cameras = GetComponentsInChildren<Camera>();
    }

    void Update()
    {
        // Update camera FOV
        foreach (Camera cam in cameras)
        {
            cam.fieldOfView = fov;
        }
    }
}
