using System.ComponentModel.Design.Serialization;
using System.ComponentModel;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace UnityStandardAssets.Vehicles.Car{
    public class CarTelemetry : MonoBehaviour{
        public CarController carController;

        [SerializeField] private float steeringAngle;
        [SerializeField] private float throttle;
        [SerializeField] private float brake;
        [SerializeField] private float speed;

        // Update is called once per frame
        void Update()
        {
            steeringAngle = carController.CurrentSteerAngle;
            throttle = carController.AccelInput;
            brake = carController.BrakeInput;
            speed = carController.CurrentSpeed;
        }
    }

}

