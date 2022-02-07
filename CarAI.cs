using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Priority_Queue;
using Assets.Scrips;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;
using Add_ab;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends; // use these to avoid collisions

        List<Vector3> my_path;
        List<Vector3> smooth_path;
        List<float> smooth_speed;

        float terrain_padding = 4f;

        /*
        public int iter;
        private int next;
        public int nextnext;
        public Vector3 start_pos;
        public Vector3 goal_pos;
        public bool Done = false;
        public bool nextPoint = false;
        public float sinceLastChange = 0;
        private Vector3 car_pos;
        private const float max_steer_angle = (25f / 360f) * 2f * Mathf.PI;
        private float theta = Mathf.PI / 2;
        private float t;
        */

        Stack<Waypoint> chosen_path;
        Waypoint current_goal;
        Pathgen pathgen;
        float driving_time_total = 0.0f;

        public int smooth_path_len;
        public int step;
        public float k_p = 2f;
        public float k_d = 0.5f;
        public float allow_error = 1.0f;
        public bool isloop;
        Rigidbody my_rigidbody;
        
        private void Start()
        {
            int a = 1;
            int b = 3;
            Class1 c = new Class1();
            c.Add_ab(a, b);

            // get the Car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            my_rigidbody = GetComponent<Rigidbody>();

            my_path = new List<Vector3>();
            pathgen = new Pathgen(terrain_manager, terrain_padding, 5f, "car");
            chosen_path = new Stack<Waypoint>(new Stack<Waypoint>(pathgen.getOptimalPath()));
            //current_goal = chosen_path.Pop();

            // linear interpolation
            int chosen_path_len = 1;
            int cps_len = 1;
            int num_interpolation;
            Waypoint current = chosen_path.Pop();
            List<Vector3> cps = new List<Vector3>(); // create contol points
            cps.Add(current.pos);
            List<Vector3> cps_new = new List<Vector3>(); // //
            int cps_new_len = 1; // //
            cps_new.Add(current.pos); // //
            // UnityEngine.Debug.Log("cps" + current.pos);
            while (chosen_path.Count > 0)
            {
                Waypoint next = chosen_path.Pop();
                float linear_dis = Vector3.Distance(current.pos, next.pos);
                //UnityEngine.Debug.Log("Linear distance" + linear_dis);
                //num_interpolation = (int)Math.Ceiling(linear_dis / 2);
                num_interpolation = 11; //  //
                //UnityEngine.Debug.Log("num_interpolation" + num_interpolation);
                for (int i=1; i<= num_interpolation; i++)
                {
                    float rate = (float) i / num_interpolation;
                    Vector3 add_point = Vector3.Lerp(current.pos, next.pos, rate);
                    cps.Add(add_point);
                    //UnityEngine.Debug.Log("cps"+ add_point);
                    if (i == 2) // //
                        cps_new.Add(add_point); // //
                    if (i == 9) // //
                        cps_new.Add(add_point); // //
                }
                cps_new_len += 2; // //
                // cps_new.Add(next.pos);// //
                // cps_new_len++; // //
                chosen_path_len++;
                cps_len += num_interpolation;
                current = next;
            }
            //UnityEngine.Debug.Log("Linear Path Length " + chosen_path_len);
            //UnityEngine.Debug.Log("Possible Control points Length " + cps_len);

            /*
            List<Vector3> cps_new = new List<Vector3>(); // create contol points
            int cps_new_len = 0;
            int interval = 4;
            for (int i=0; i< cps_len-1; i++)
            {
                if ((i % interval) == 0)
                {
                    cps_new.Add(cps[i]);
                    //UnityEngine.Debug.Log("cps_new" + cps[i]);
                    cps_new_len++;
                }
            }
            cps_new.Add(cps[cps_len-1]);
            //UnityEngine.Debug.Log("cps_new" + cps[cps_len - 1]);
            cps_new_len++;
            //UnityEngine.Debug.Log("Control points Length" + cps_new_len);
            */
            Vector3 old_cp = cps_new[0];
            foreach (var cp in cps_new)
            {
                Debug.DrawLine(old_cp, cp, Color.red, 100f);
                old_cp = cp;
            }

            // Smoothness
            SplineCurve curve = new SplineCurve();
            foreach (var cp in cps_new)
            {
                curve.AddNode(cp);
            }
            curve.AddCatmull_RomControl();
            smooth_path = new List<Vector3>(); //create smooth path
            for (int i = 0; i < curve.segmentList.Count; i++)
            {
                float add = 1f / 10;  // 表示两个关键点之间取20个点，可根据需要设置
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

            
            List<float> curvature = new List<float>();
            smooth_speed = new List<float>();
            float yaw_rate = 0.05f;
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
                    if (smooth_speed[i] > 0.1f)
                        smooth_speed[i] = 0.1f;
                    //if (smooth_speed[i] < 1.0f)
                    //    smooth_speed[i] = 1.0f;
                }
                UnityEngine.Debug.Log("Curvature" + curvature[i] + "Smooth speed" + smooth_speed[i]);
            }
            

            // initialize the control
            step = 1;
        }

        public static Vector3[] GetBezierCurveWithThreePoints(Vector3 point_1, Vector3 point_2, Vector3 point_3, int vertexCount)
        {
            List<Vector3> pointList = new List<Vector3>();
            for (float ratio = 0; ratio <= 1; ratio += 1.0f / vertexCount)
            {
                //首先取前两个点和后两个点的线性插值。

                Vector3 tangentLineVertex1 = Vector3.Lerp(point_1, point_2, ratio);
                Vector3 tangentLineVertex2 = Vector3.Lerp(point_2, point_3, ratio);
                //通过计算两个点的插值得到曲线的顶点

                Vector3 bezierPoint = Vector3.Lerp(tangentLineVertex1, tangentLineVertex2, ratio);
                pointList.Add(bezierPoint);
            }
            pointList.Add(point_3);
            return pointList.ToArray();
        }

        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            


            
            Vector3 target_position = smooth_path[step];
            Vector3 current_position = transform.position;

            if (step < smooth_path_len - 1)
            {
                if (Vector3.Distance(target_position, current_position) < allow_error)
                {
                    step++;
                    target_position = smooth_path[step];
                }
            }

            if (step == smooth_path_len - 1)
            {
                //m_Car.Move(0f, 0f, 0f, 1f);
                print("Arrive");
                return;
                /*
                if (Vector3.Distance(target_position, current_position) < allow_error)
                {
                    print("Arrive");
                    return;
                }
                */
            }
            /*
            if (Vector3.Distance(target_position, current_position) < allow_error)
            {
                if (step < smooth_path_len - 1)
                {
                    step++;
                    target_position = smooth_path[step];
                }
                else
                    return;
            }
            */

            // calculate traget velocity and steering angle


            //float turn_angle_ratio = Mathf.Pow(Vector3.Angle(target_position - smooth_path[step - 1], smooth_path[step + 1] - target_position), 5);
            //UnityEngine.Debug.Log("turn_angle_ratio" + turn_angle_ratio);
            /*
            Vector3 target_velocity = (smooth_path[step + 1] - target_position) / Time.fixedDeltaTime;
            if (step == smooth_path_len - 1)
            {
                target_velocity = (target_position - target_position) / Time.fixedDeltaTime;
            }
            */

            //Vector3 target_velocity = (target_position- smooth_path[step -1]) / Time.fixedDeltaTime;
            Vector3 target_velocity = ((target_position - smooth_path[step - 1]) / Time.fixedDeltaTime).normalized * smooth_speed[step];
            Vector3 position_error = target_position - current_position;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            /*
            if (step > smooth_path_len - 10)
            {
                float rate = (float)1.0f / (step - smooth_path_len + 10);
                Debug.Log("Step" + step + " Rate:" + rate);
                steering = steering * rate;
                acceleration = acceleration * rate;
            }
            */
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);


            /*
            if (step < smooth_path_len - 25)
                m_Car.Move(steering, acceleration, acceleration, 0f);
                driving_time_total += Time.fixedDeltaTime;

            if (step >= smooth_path_len - 25)
                m_Car.Move(steering, 0.0f, -1.0f, 0f);
                driving_time_total += Time.fixedDeltaTime;
            //print(driving_time_total);
            //print(step);
            */
            

            /*
                // this is how you access information about the terrain from the map
                int i = terrain_manager.myInfo.get_i_index(transform.position.x);
                int j = terrain_manager.myInfo.get_j_index(transform.position.z);
                float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
                float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

                Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

                // this is how you access information about the terrain from a simulated laser range finder
                RaycastHit hit;
                float maxRange = 50f;
                if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
                {
                    Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                    Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    Debug.Log("Did Hit");
                }
            */
        }
        
    }
}
            
