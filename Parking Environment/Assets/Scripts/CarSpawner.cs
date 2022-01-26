using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarSpawner : MonoBehaviour
{
    public float DrivingDistance = 20f;
    public float DrivingSpeed = 1.5f; // Speed in m/s
    public float DrivingSpeedVariance = 1.5f; // Speed variance in m/s
    public float SpawnTimer = 2f;
    public float SpawnTimerVariance = 3f;

    private float timer = 0f;
    
    public GameObject InitialCar;
    private List<GameObject> Cars = new List<GameObject>();

    private Vector3 SpawnPosition;
    private Quaternion SpawnRotation;

    // Start is called before the first frame update
    void Start()
    {
        SpawnPosition = gameObject.transform.position;
        SpawnRotation = gameObject.transform.rotation;

        SpawnCar();
    }

    // Update is called once per frame
    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= SpawnTimer + Random.Range(0, SpawnTimerVariance))
        {
            timer = 0f;
            SpawnCar();
        }


        foreach (GameObject Car in Cars)
        {
            float DistanceDriven = Mathf.Abs(SpawnPosition.x - Car.transform.position.x);
            
            if (DistanceDriven > DrivingDistance)
            {
                DeleteCar(Car);
                break;
            }
            else
            {
                float speed = DrivingSpeed + Random.Range(-DrivingSpeedVariance, DrivingSpeedVariance);
                Car.transform.Translate(Vector3.forward * speed * Time.deltaTime);
            }

            
        }
    }


    private void SpawnCar()
    {
        GameObject newCar = Instantiate(InitialCar, SpawnPosition, SpawnRotation);
        Cars.Add(newCar);
    }

    private void DeleteCar(GameObject Car)
    {
        Cars.Remove(Car);
        Destroy(Car);
    }
}

