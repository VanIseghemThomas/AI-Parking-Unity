
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarStripSpawner : MonoBehaviour
{
    public int numberOfCarsToSpawn = 20;
    public GameObject[] Cars;
    [Range(0,1)] public float spotFrequency = .15f;
    public float spotSize = 8f;
    public float carMargin = 1f;


    private GameObject initalCar;
    private float initalCarLength;

    private float currentSpawnPosition;


    // Start is called before the first frame update
    void Start()
    {
        // Get child gameobject
        initalCar = transform.GetChild(0).gameObject;

        initalCarLength = GetCarLength(initalCar);
        
        Debug.Log(initalCarLength);  

        // Set the current spawn position to the next available spot
        currentSpawnPosition = initalCar.transform.localPosition.z + initalCarLength + carMargin;     

        GenerateCars(); 
    }

    float GetCarLength(GameObject car)
    {
        // This indicates how long the vehicle is
        GameObject carBody = car.transform.GetChild(0).gameObject;
        float carLength = carBody.GetComponent<MeshRenderer>().bounds.size.z;

        return carLength;
    }

    void GenerateCars(){
        for (int i = 0; i < numberOfCarsToSpawn; i++)
        {
            // Get a random car
            int randomCarIndex = Random.Range(0, Cars.Length);
            GameObject randomCar = Cars[randomCarIndex];
            float carLength = GetCarLength(randomCar);

            currentSpawnPosition += carLength + carMargin;

            // Take random choice to determine if a car should spawn or a spot should be left open
            if (Random.value < spotFrequency)
            {
                // Car length gets replaced by predefined spot size
                // Also double the car margin
                currentSpawnPosition += spotSize - carLength;
                //Spawn an empty gameobject with a collider trigger
                GameObject spot = new GameObject();
                spot.transform.SetParent(this.transform);
                spot.transform.localPosition = new Vector3(0, 0, currentSpawnPosition) + initalCar.transform.localPosition;
                BoxCollider trigger = spot.AddComponent<BoxCollider>();
                trigger.isTrigger = true;
                trigger.size = new Vector3(20, spotSize, spotSize);

            }
            else
            {
                // Spawn a car
                GameObject newCar = Instantiate(randomCar, new Vector3(0, 0, currentSpawnPosition), Quaternion.identity);
                // Set Parent to this gameobject
                newCar.transform.SetParent(this.transform);
                // Adjust transform to be relative to this gameobject
                newCar.transform.localPosition = new Vector3(0, 0, currentSpawnPosition) + initalCar.transform.localPosition;
            }
        }
    }
}
