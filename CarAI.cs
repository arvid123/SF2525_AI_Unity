using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Priority_Queue;
using Assets.Scrips;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends; // use these to avoid collisions

        float terrain_padding = 4f;
        Pathgen pathgen;

        List<Vector3> bezier_path;
        public int bezier_path_len;
        List<Vector3> smooth_path;
        public int smooth_path_len;
        List<float> smooth_speed;

        public float dis_interval = 2.0f; // distance interval between each pont in linear interpolation
        public int knot_interval = 20;  // downsampling to get knots
        public int spline_resolution = 10; // number of points between knots
        public float yaw_rate = 50.0f;
        public float speed_max = 50.0f;
        public float speed_min = 0.0f;

        public int step;
        public float k_p = 2f;
        public float k_d = 0.5f;
        public float error_threshold_min = 5.0f;
        public float error_threshold_max = 10.0f;
        public int forward_step = 10; // search step number for control error correction
        Rigidbody my_rigidbody;
        
        private void Start()
        {
            // get the Car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            my_rigidbody = GetComponent<Rigidbody>();

            pathgen = new Pathgen(terrain_manager, terrain_padding, 5f, "car");
            bezier_path = pathgen.getBezierPathList();
            bezier_path_len = bezier_path.Count;
            // UnityEngine.Debug.Log("bezier_path_len" + bezier_path_len);
            for (int i = 0; i < bezier_path_len - 1; i++)
            {
                //Debug.DrawLine(bezier_path[i], bezier_path[i+1], Color.yellow, 10000f);
            }

            //linear interpolation
            int num_interpolation;
            List<Vector3> cps = new List<Vector3>(); // create contol points
            cps.Add(bezier_path[0]);
            int cps_len = 1;
            for (int j = 0; j < bezier_path_len-1; j++)
            {
                float linear_dis = Vector3.Distance(bezier_path[j], bezier_path[j+1]);
                num_interpolation = (int)Math.Ceiling(linear_dis / dis_interval);
                for (int i = 1; i <= num_interpolation; i++)
                {
                    float rate = (float)i / num_interpolation;
                    Vector3 add_point = Vector3.Lerp(bezier_path[j], bezier_path[j + 1], rate);
                    cps.Add(add_point);
                }
                cps_len += num_interpolation;
            }
            // UnityEngine.Debug.Log("Possible Control points Length " + cps_len);

            List<Vector3> cps_new = new List<Vector3>(); // create contol points
            int cps_new_len = 0;
            for (int i = 0; i < cps_len - 1; i++)
            {
                if ((i % knot_interval) == 0)
                {
                    cps_new.Add(cps[i]);
                    cps_new_len++;
                }
            }
            cps_new.Add(cps[cps_len - 1]);
            cps_new_len++;
            //UnityEngine.Debug.Log("Control points Length" + cps_new_len);

            /*
            Vector3 old_cp = cps_new[0];
            foreach (var cp in cps_new)
            {
                Debug.DrawLine(old_cp, cp, Color.red, 100f);
                old_cp = cp;
            }
            */

            // Spline Smoothness
            SplineCurve curve = new SplineCurve();
            foreach (var cp in cps_new)
            {
                curve.AddNode(cp);
            }
            curve.AddCatmull_RomControl();
            smooth_path = new List<Vector3>(); //create smooth path
            for (int i = 0; i < curve.segmentList.Count; i++)
            {
                float add = 1f / spline_resolution;  // 表示两个关键点之间取20个点，可根据需要设置
                for (float j = 0; j < 1; j += add)
                {
                    Vector3 point = curve.segmentList[i].GetPoint(j);
                    smooth_path.Add(point);
                }
            }

            Vector3 old_sp = smooth_path[0];
            smooth_path_len = 0;
            foreach (var sp in smooth_path)
            {
                Debug.DrawLine(old_sp, sp, Color.cyan, 100f);
                old_sp = sp;
                smooth_path_len++;
            }
            //UnityEngine.Debug.Log("Smooth Path Length " + smooth_path_len);

            // Curvature
            List<float> curvature = new List<float>();
            smooth_speed = new List<float>();
            for (int i = 0; i< smooth_path_len; i++)
            {
                if (i == 0)
                {
                    curvature.Add(Vector3.Angle(smooth_path[i] - smooth_path[i], smooth_path[i + 1] - smooth_path[i]));
                    smooth_speed.Add(0);
                }
                else if (i == (smooth_path_len - 1))
                {
                    curvature.Add(Vector3.Angle(smooth_path[i] - smooth_path[i - 1], smooth_path[i] - smooth_path[i]));
                    smooth_speed.Add(0);
                }
                else
                {
                    curvature.Add(Vector3.Angle(smooth_path[i] - smooth_path[i - 1], smooth_path[i + 1] - smooth_path[i]));
                    smooth_speed.Add((1.0f / curvature[i]) * yaw_rate);
                    if (smooth_speed[i] > speed_max)
                        smooth_speed[i] = speed_max;
                    //if (smooth_speed[i] < speed_min)
                    //    smooth_speed[i] = speed_min;
                }
                UnityEngine.Debug.Log("Curvature" + curvature[i] + "Smooth speed" + smooth_speed[i]);
            }
            // initialize the control
            step = 1;
        }

        private void FixedUpdate()
        {
            // Execute your path here
            // ...
            Vector3 target_position = smooth_path[step];
            Vector3 current_position = transform.position;

            if (step < smooth_path_len - 1)
            {
                if (Vector3.Distance(target_position, current_position) < error_threshold_min)
                {
                    step++;
                    target_position = smooth_path[step];
                }
                else if (Vector3.Distance(target_position, current_position) > error_threshold_max)
                {
                    float value = Vector3.Dot((smooth_path[step] - smooth_path[step - 1]).normalized, my_rigidbody.velocity.normalized);
                    float value_temp;
                    int temp = step;
                    for (int i = step; i< step + forward_step; i++)
                    {
                        if (i >= smooth_path_len)
                            break;
                        value_temp = Vector3.Dot(smooth_path[i] - smooth_path[i - 1], my_rigidbody.velocity);
                        if (value_temp > value)
                        {
                            temp = i;
                            value = value_temp;
                        }
                    }
                    step = temp;
                    target_position = smooth_path[step];
                }
                Debug.DrawLine(smooth_path[step-1], target_position, Color.black, 100f);
            }

            if (step == smooth_path_len - 1)
            {
                print("Arrive");
                m_Car.Move(0.0f, 0.0f, -1.0f, 1.0f);
                return;
            }

            Vector3 target_velocity = ((target_position - smooth_path[step - 1]) / Time.fixedDeltaTime).normalized * smooth_speed[step];
            Vector3 position_error = (target_position - current_position).normalized;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }
        
    }
}
            
