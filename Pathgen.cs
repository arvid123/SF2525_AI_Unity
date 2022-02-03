using Priority_Queue;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scrips
{
    class Pathgen
    {
        TerrainManager terrain_manager;
        float terrain_padding;
        List<Vector3> my_path;
        Stack<Waypoint> optimal_path;
        float max_turning_velocity;

        public Pathgen(TerrainManager t, float tp, float mtv)
        {
            terrain_manager = t;
            terrain_padding = tp;
            max_turning_velocity = mtv;

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            my_path = new List<Vector3>();

            // Plan your path here
            // ...

            //for (int i = 0; i < 3; i++)
            //{
            //}


            // Create padded visibility graph
            List<GameObject> padded_cubes = new List<GameObject>();
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
                        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        cube.transform.position = new Vector3(ter.get_x_pos(i), 0.0f, ter.get_z_pos(j));
                        cube.transform.localScale = new Vector3(x_step + terrain_padding * 1.9f, 15.0f, z_step + terrain_padding * 1.9f);
                        cube.GetComponent<MeshRenderer>().enabled = false;
                        cube.layer = 1;
                        padded_cubes.Add(cube);

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
                    if (!w.pos.Equals(otherw.pos) && !Physics.Linecast(w.pos, otherw.pos) || ((w.Equals(goal) || otherw.Equals(goal)) && !Physics.Linecast(w.pos, otherw.pos, LayerMask.GetMask("Default"))))
                    {
                        w.neighbors.Add(otherw);
                        Debug.DrawLine(w.pos, otherw.pos, Color.red, 100f);
                    }
                }
            }

            // Find optimal path and draw it
            optimal_path = A_star(start, goal, wps);
            Stack<Waypoint> path = new Stack<Waypoint>(new Stack<Waypoint>(optimal_path));
            
            Debug.Log(path.Count);
            Waypoint current = path.Pop();
            while (path.Count > 0)
            {
                Waypoint next = path.Pop();
                Debug.Log(current.drone_goal_vel);
                Debug.DrawLine(current.pos, next.pos, Color.yellow, 1000f);
                current = next;
            }

            // Delete padded cubes
            foreach (var cube in padded_cubes)
            {
                cube.SetActive(false);
            }
        }

        public Stack<Waypoint> getOptimalPath()
        {
            return new Stack<Waypoint>(new Stack<Waypoint>(optimal_path));
        }

        // https://en.wikipedia.org/wiki/A*_search_algorithm 
        public Stack<Waypoint> A_star(Waypoint start, Waypoint goal, List<Waypoint> waypoints)
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
            Debug.Log("A* failed");
            return null;
        }

        // The heuristic for the A* algorithm
        private double h(Waypoint w, Waypoint goal)
        {
            return Vector3.Distance(w.pos, goal.pos);
        }

        // Create stack of waypoints and also add drone_goal_vel depending on the angle between the points
        private Stack<Waypoint> reconstruct_path(Waypoint start, Waypoint goal)
        {
            Stack<Waypoint> path = new Stack<Waypoint>();
            Waypoint current = goal;
            goal.drone_goal_vel = 0;
            start.drone_goal_vel = 0;

            while (current.cameFrom.cameFrom != null)
                {
                Waypoint next = current.cameFrom;
                Waypoint next_next = next.cameFrom;

                float turn_angle_ratio = Mathf.Pow(Vector3.Angle(current.pos - next.pos, next_next.pos - next.pos) / 180f, 4);
                next.drone_goal_vel = max_turning_velocity * turn_angle_ratio;

                path.Push(current);
                current = next;
            }
            path.Push(current);
            path.Push(current.cameFrom);

            return path;
        }

        private float squared(float x)
        {
            return x * x;
        }

        private float cubed(float x)
        {
            return x * x * x;
        }

        public Stack<Waypoint> getSmoothPath()
        {
            var chosen_path = getOptimalPath();
            List<Vector3> smooth_path;
            // linear interpolation
            int chosen_path_len = 1;
            int cps_len = 1;
            int num_interpolation;
            Waypoint current = chosen_path.Pop();
            List<Vector3> cps = new List<Vector3>(); // create contol points
            cps.Add(current.pos);
            // UnityEngine.Debug.Log("cps" + current.pos);
            while (chosen_path.Count > 0)
            {
                Waypoint next = chosen_path.Pop();
                float linear_dis = Vector3.Distance(current.pos, next.pos);
                //UnityEngine.Debug.Log("Linear distance" + linear_dis);
                num_interpolation = (int)Math.Ceiling(linear_dis / 4);
                UnityEngine.Debug.Log("num_interpolation" + num_interpolation);
                for (int i = 1; i <= num_interpolation; i++)
                {
                    float rate = (float)i / num_interpolation;
                    Vector3 add_point = Vector3.Lerp(current.pos, next.pos, rate);
                    cps.Add(add_point);
                    //UnityEngine.Debug.Log("cps"+ add_point);
                }
                chosen_path_len++;
                cps_len += num_interpolation;
                current = next;
            }
            //UnityEngine.Debug.Log("Linear Path Length " + chosen_path_len);
            //UnityEngine.Debug.Log("Possible Control points Length " + cps_len);

            List<Vector3> cps_new = new List<Vector3>(); // create contol points
            int cps_new_len = 0;
            int interval = 4;
            for (int i = 0; i < cps_len - 1; i++)
            {
                if ((i % interval) == 0)
                {
                    cps_new.Add(cps[i]);
                    //UnityEngine.Debug.Log("cps_new" + cps[i]);
                    cps_new_len++;
                }
            }
            cps_new.Add(cps[cps_len - 1]);
            //UnityEngine.Debug.Log("cps_new" + cps[cps_len - 1]);
            cps_new_len++;
            //UnityEngine.Debug.Log("Control points Length" + cps_new_len);

            Vector3 old_cp = cps_new[0];
            foreach (var cp in cps_new)
            {
                //Debug.DrawLine(old_cp, cp, Color.red, 100f);
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
            foreach (var sp in smooth_path)
            {
                Debug.DrawLine(old_sp, sp, Color.blue, 100f);
                old_sp = sp;
            }
            //UnityEngine.Debug.Log("Smooth Path Length " + smooth_path_len);

            Stack<Waypoint> path = new Stack<Waypoint>(smooth_path.ConvertAll<Waypoint>(x => new Waypoint(x)));
            Waypoint current_point = path.Pop();
            Waypoint next_point = path.Pop();
            Waypoint next_next = path.Pop();
            Stack<Waypoint> smooth_stack = new Stack<Waypoint>();

            do
            {

                float turn_angle_ratio = Vector3.Angle(current_point.pos - next_point.pos, next_next.pos - next_point.pos) / 180f;
                next_point.drone_goal_vel = max_turning_velocity * turn_angle_ratio;

                smooth_stack.Push(current_point);
                current_point = next_point;
                next_point = next_next;
                next_next = path.Pop();
            } while (path.Count > 0);
            smooth_stack.Push(next_next);

            return smooth_stack;
        }

    }
}
