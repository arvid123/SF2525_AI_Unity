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

        List<Vector3> my_path;
        List<Vector3> smooth_path;
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

        public int path_length;
        public int step = 0;
        public float k_p = 2f;
        public float k_d = 0.5f;
        public float allow_error = 2.0f;
        public bool isloop;
        // Rigidbody my_rigidbody;
        
        private void Start()
        {
            // get the drone controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            // my_rigidbody = GetComponent<Rigidbody>();

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            my_path = new List<Vector3>();

            // Plan your path here
            // ...

            //for (int i = 0; i < 3; i++)
            //{
            //}


            // Create padded visibility graph
            var ter = terrain_manager.myInfo;
            float x_step = (ter.x_high - ter.x_low) / ter.x_N;
            float z_step = (ter.z_high - ter.z_low) / ter.z_N;
            for (int i = 0; i < ter.x_N; i++)
            {
                for (int j = 0; j < ter.z_N; j++)
                {
                    if (ter.traversability[i, j] > 0.5f)
                    {
                        // Add invisible padded cube for collision detection
                        /*GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        cube.transform.position = new Vector3(ter.get_x_pos(i), 0.0f, ter.get_z_pos(j));
                        cube.transform.localScale = new Vector3(x_step + terrain_padding*1.9f, 15.0f, z_step + terrain_padding*1.9f);*/
                        Vector3 lower_left = new Vector3(ter.get_x_pos(i) - x_step / 2 - terrain_padding, 0, ter.get_z_pos(j) - z_step / 2 - terrain_padding);
                        if (ter.traversability[ter.get_i_index(lower_left.x), ter.get_j_index(lower_left.z)] <= 0.5f)
                        {
                            // Make sure we only add convex corners to the graph
                            if (j > 0 && ter.traversability[i, j - 1] <= 0.5f && i > 0 && ter.traversability[i - 1, j] <= 0.5f)
                            {
                                my_path.Add(lower_left);
                            }
                        }
                        Vector3 lower_right = new Vector3(ter.get_x_pos(i) + x_step / 2 + terrain_padding, 0, ter.get_z_pos(j) - z_step / 2 - terrain_padding);
                        if (ter.traversability[ter.get_i_index(lower_right.x), ter.get_j_index(lower_right.z)] <= 0.5f)
                        {
                            // Make sure we only add convex corners to the graph
                            if (j > 0 && ter.traversability[i, j - 1] <= 0.5f && i < ter.x_N - 1 && ter.traversability[i + 1, j] <= 0.5f)
                            {
                                my_path.Add(lower_right);
                            }
                        }
                        Vector3 upper_left = new Vector3(ter.get_x_pos(i) - x_step / 2 - terrain_padding, 0, ter.get_z_pos(j) + z_step / 2 + terrain_padding);
                        if (ter.traversability[ter.get_i_index(upper_left.x), ter.get_j_index(upper_left.z)] <= 0.5f)
                        {
                            // Make sure we only add convex corners to the graph
                            if (j < ter.z_N - 1 && ter.traversability[i, j + 1] <= 0.5f && i > 0 && ter.traversability[i - 1, j] <= 0.5f)
                            {
                                my_path.Add(upper_left);
                            }
                        }
                        Vector3 upper_right = new Vector3(ter.get_x_pos(i) + x_step / 2 + terrain_padding, 0, ter.get_z_pos(j) + z_step / 2 + terrain_padding);
                        if (ter.traversability[ter.get_i_index(upper_right.x), ter.get_j_index(upper_right.z)] <= 0.5f)
                        {
                            // Make sure we only add convex corners to the graph
                            if (j < ter.z_N - 1 && ter.traversability[i, j + 1] <= 0.5f && i < ter.x_N - 1 && ter.traversability[i + 1, j] <= 0.5f)
                            {
                                my_path.Add(upper_right);
                            }
                        }
                    }

                }
            }


            /* Plot your path to see if it makes sense
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
            */

            List<Waypoint> wps = new List<Waypoint>();

            foreach (var vec in my_path)
            {
                Waypoint w = new Waypoint(vec);
                wps.Add(w);
            }

            Waypoint start = new Waypoint(start_pos);
            Waypoint goal = new Waypoint(goal_pos);
            wps.Add(start);
            wps.Add(goal);

            foreach (var w in wps)
            {
                foreach (var otherw in wps)
                {
                    if (!Physics.Linecast(w.pos, otherw.pos) && !w.pos.Equals(otherw.pos))
                    {
                        w.neighbors.Add(otherw);
                        Debug.DrawLine(w.pos, otherw.pos, Color.red, 2f);
                    }
                }
            }

            // Find optimal path and draw it
            Stack<Waypoint> optimal_path = A_star(start, goal, wps);
            Debug.Log(optimal_path.Count);
            Waypoint current = optimal_path.Pop();
            List<Vector3> cps= new List<Vector3>(); // create contol points
            cps.Add(current.pos);
            while (optimal_path.Count > 0)
            {
                Waypoint next = optimal_path.Pop();
                Debug.DrawLine(current.pos, next.pos, Color.yellow, 1000f);
                current = next;
                cps.Add(current.pos);
            }

            // Find smooth path
            SplineCurve curve = new SplineCurve();
            foreach (var cp in cps)
            {
                curve.AddNode(cp);
            }
            curve.AddCatmull_RomControl();
            smooth_path = new List<Vector3>(); //create smooth path
            for (int i = 0; i < curve.segmentList.Count; i++)
            {
                float add = 1f / 20;  // 表示两个关键点之间取20个点，可根据需要设置
                for (float j = 0; j < 1; j += add)
                {
                    Vector3 point = curve.segmentList[i].GetPoint(j);
                    smooth_path.Add(point);
                }
            }

            Vector3 old_sp = start_pos;
            path_length = 0;
            foreach (var sp in smooth_path)
            {
                Debug.DrawLine(old_sp, sp, Color.blue, 100f);
                old_sp = sp;
                path_length = path_length + 1;
            }
            UnityEngine.Debug.Log(" Path Length " + path_length);

        }



        // https://en.wikipedia.org/wiki/A*_search_algorithm 
        private Stack<Waypoint> A_star(Waypoint start, Waypoint goal, List<Waypoint> waypoints)
        {
            SimplePriorityQueue<Waypoint, double> openSet = new SimplePriorityQueue<Waypoint, double>();
            openSet.Enqueue(start, double.PositiveInfinity);
            start.gScore = 0;

            while (openSet.Count > 0)
            {
                Waypoint current = openSet.Dequeue();
                if (current.Equals(goal))
                {
                    return reconstruct_path(start, goal);
                }

                foreach (var neighbor in current.neighbors)
                {
                    double tentative_gScore = current.gScore + Vector3.Distance(current.pos, neighbor.pos);
                    if (tentative_gScore < neighbor.gScore)
                    {
                        // This path to neighbor is better than any other recorded one.
                        neighbor.cameFrom = current;
                        neighbor.gScore = tentative_gScore;
                        openSet.EnqueueWithoutDuplicates(neighbor, tentative_gScore + h(neighbor, goal));
                    }
                }
            }

            // Should be unreachable
            return null;
        }

        // The heuristic for the A* algorithm
        private double h(Waypoint w, Waypoint goal)
        {
            return Vector3.Distance(w.pos, goal.pos);
        }

        private Stack<Waypoint> reconstruct_path(Waypoint start, Waypoint goal)
        {
            Stack<Waypoint> path = new Stack<Waypoint>();
            Waypoint current = goal;
            while (!current.Equals(start))
            {
                path.Push(current);
                current = current.cameFrom;
            }
            path.Push(current);

            return path;
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
            /*
            iter++;
            sinceLastChange += m_Car.CurrentSpeed * t;
            if (Done)
            {
                m_Car.Move(0f, 0f, 0f, 1f);
            }
            else
            {
                //UnityEngine.Debug.Log("----------------------------- Iteration: " + iter + " -----------------------------");
                if (nextPoint)
                {
                    sinceLastChange += 1;
                    if (sinceLastChange > 4)
                    {
                        next++;
                        sinceLastChange = 0;
                        nextPoint = false;
                        if (nextnext <= next)
                        {
                            nextnext++;
                        }
                    }
                }
                car_pos = new Vector3(transform.position.x, 0, transform.position.z);
                //UnityEngine.Debug.Log(next + "/" + my_path.Count);
                float[] results = Steer(car_pos, smooth_path[next]);
                float steer = -results[1] / max_steer_angle;
                //UnityEngine.Debug.Log(" Steer: " + steer + " velocity: " + m_Car.CurrentSpeed + " accel: " + results[0] + " Brake: " + results[2]);
                m_Car.Move(steer, results[0], 0f, results[2]);
            }
            */

            // Execute your path here
            // ...
            Vector3 target_position = smooth_path[step];
            Vector3 current_position = transform.position;
            //if 
            //step = step + 1;
            print(step);
            
            
            //Vector3 target_velocity = (target_position - current_position) / Time.fixedDeltaTime;

            // Vector3 position_error = target_position - transform.position;
            //Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            //Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            //float steering = Vector3.Dot(desired_acceleration, transform.right);
            //float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(steering, acceleration, acceleration, 0f);

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
            // this is how you control the car
            //m_Car.Move(1f, 1f, 1f, 0f);
        }
        /*
        private float[] Steer(Vector3 curPos, Vector3 pos)
        {
            t = Time.fixedDeltaTime;
            float Accel = 1f;
            float footbreak = 0f;

            theta = nfmod(-m_Car.transform.eulerAngles.y * (Mathf.PI / 180.0f) + Mathf.PI / 2.0f, 2 * Mathf.PI);
            float v = m_Car.CurrentSpeed;

            float xDist = pos[0] - curPos[0];
            float zDist = pos[2] - curPos[2];

            float newDistance = Mathf.Sqrt(Mathf.Pow(xDist, 2) + Mathf.Pow(zDist, 2));
            float goalDistance = Mathf.Sqrt(Mathf.Pow(curPos[0] - goal_pos[0], 2) + Mathf.Pow(curPos[2] - goal_pos[2], 2));
            if (newDistance < 5)
            {
                nextPoint = true;

            }
            if (newDistance < 1)
            {
                next++;
                nextPoint = false;
                sinceLastChange = 0;
                if (nextnext <= next)
                {
                    nextnext = next + 1;
                }
            }
            if (goalDistance < 10)
            {
                Done = true;
                float[] ret = { 0f, 0f, 0f };
                return ret;
            }

            float vectorAngle = Mathf.Atan(zDist / xDist);
            if (
                (xDist < 0 && zDist > 0 && vectorAngle < 0) // if car_next lies in the second quadrant
                || (xDist < 0 && vectorAngle == 0) // if car_next lies on the x axis and points to the left
                || (xDist < 0 && zDist < 0) // if car_next lies in the third quadrant
            )
            {
                vectorAngle += Mathf.PI;
            }
            else if (xDist > 0 && zDist < 0)// if car_next lies in the fourth quadrant
            {
                vectorAngle += 2 * Mathf.PI;
            }

            float beta = 0;
            if (vectorAngle < Mathf.PI / 2)
            {
                //UnityEngine.Debug.Log("Vector in the first quadrant");
                if (theta < Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = -vectorAngle;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = 2 * Mathf.PI + vectorAngle - theta;
                }
            }
            else if (vectorAngle < Mathf.PI)
            {
                //UnityEngine.Debug.Log("Vector in the second quadrant");
                if (theta < Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = -vectorAngle;
                }
            }
            else if (vectorAngle < 3 * Mathf.PI / 2)
            {
                //UnityEngine.Debug.Log("Vector in the third quadrant");
                if (theta < Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = vectorAngle;
                }
                else if (theta < Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = vectorAngle - theta;
                }
            }
            else if (vectorAngle < 2 * Mathf.PI)
            {
                //UnityEngine.Debug.Log("Vector in the fourth quadrant");
                if (theta < Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the first quadrant");
                    beta = -2 * Mathf.PI - vectorAngle - theta;
                }
                else if (theta < Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the second quadrant");
                    beta = -(Mathf.Sign(vectorAngle - Mathf.PI - theta));
                }
                else if (theta < 3 * Mathf.PI / 2)
                {
                    //UnityEngine.Debug.Log("Theta in the third quadrant");
                    beta = vectorAngle - theta;
                }
                else if (theta < 2 * Mathf.PI)
                {
                    //UnityEngine.Debug.Log("Theta in the fourth quadrant");
                    beta = vectorAngle - theta;
                }
            }
            else
            {
                //UnityEngine.Debug.Log("None of the above. Should not happen.");
                beta = vectorAngle - theta;
            }

            if (nextnext < smooth_path.Count)
            {
                float nextnextX = smooth_path[nextnext][0] - curPos[0];
                float nextnextZ = smooth_path[nextnext][2] - curPos[2];
                float nextnextdistance = Mathf.Sqrt(Mathf.Pow(nextnextX, 2) + Mathf.Pow(nextnextZ, 2));
                if (nextnextdistance < 10)
                {
                    nextnext++;
                }
                float xNext = smooth_path[nextnext][0] - pos[0];
                float zNext = smooth_path[nextnext][2] - pos[2];
                float nextAngle = Mathf.Atan(zNext / xNext);
                float nextBeta = 0;
                if (
                    (xNext < 0 && zNext > 0 && nextAngle < 0) // if car_next lies in the second quadrant
                    || (xNext < 0 && nextAngle == 0) // if car_next lies on the x axis and points to the left
                    || (xNext < 0 && zNext < 0) // if car_next lies in the third quadrant
                )
                {
                    nextAngle += Mathf.PI;
                }
                else if (xNext > 0 && zNext < 0)// if car_next lies in the fourth quadrant
                {
                    nextAngle += 2 * Mathf.PI;
                }
                nextBeta = theta - nextAngle;

                if (v < 30)
                {
                    Accel = 1f;
                    footbreak = 0;
                }
                else if (nextBeta < Mathf.PI / 4 && nextBeta > -Mathf.PI / 4)
                {
                    Accel = 1f;
                    if (newDistance < 20)
                    {
                        Accel = 0f;
                    }
                    //UnityEngine.Debug.Log("Next beta less than PI/4");
                }
                else if (nextBeta < Mathf.PI / 2 && nextBeta > -Mathf.PI / 2)
                {
                    if (newDistance > 30)
                    {
                        Accel = 0f;
                    }
                    else
                    {
                        Accel = 0f;
                        footbreak = 1f;
                    }
                    //UnityEngine.Debug.Log("Next beta less than PI/2");
                }
                else if (nextBeta < 3 * Mathf.PI / 4 && nextBeta > -3 * Mathf.PI / 4)
                {
                    if (newDistance > 40)
                    {
                        Accel = 0;
                    }
                    else
                    {
                        Accel = 0;
                        footbreak = 1f;
                    }
                    //UnityEngine.Debug.Log("Next beta more than PI/2");
                }
            }
            if (beta < -max_steer_angle)
            {
                beta = -max_steer_angle;
            }
            else if (beta > max_steer_angle)
            {
                beta = max_steer_angle;
            }
            float[] results = { Accel, beta, footbreak };
            return results;
        }
        public float nfmod(float a, float b)
        {
            return a - b * Mathf.Floor(a / b);
        }
        */
    }
}
