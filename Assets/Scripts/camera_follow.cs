using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace simple_case
{
    public class camera_follow : MonoBehaviour
    {
        private Transform our_drone;
        void Awake()
        {
            // our_drone = GameObject.FindGameObjectWithTag("player").transform;
        }
        
        private Vector3 velocity_camera_follow;
        private Vector3 behind_position = new Vector3(0, 2, -4);
        public float angle;
        void FixedUpdate()
        {
            // transform.position = Vector3.SmoothDamp(transform.position, our_drone.transform.TransformPoint(behind_position)+Vector3.up * Input.GetAxis("z-axis"), ref velocity_camera_follow, 0.1f);
            // transform.rotation = Quaternion.Euler(new Vector3(angle, our_drone.GetComponent<drone_controller>().current_y_rotation, 0));
        }
    }
}