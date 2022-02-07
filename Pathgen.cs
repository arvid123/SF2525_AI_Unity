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

        public Pathgen(TerrainManager t, float tp, float mtv, string vehicle)
        {
            terrain_manager = t;
            terrain_padding = tp;
            max_turning_velocity = mtv;

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            my_path = new List<Vector3>();
            Debug.DrawRay(start_pos, Vector3.forward, Color.magenta, 1000f);

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

            bool goal_within_padding = Physics.OverlapSphere(goal_pos, 0, LayerMask.GetMask("TransparentFX")).Length > 0;

            foreach (var w in wps)
            {
                foreach (var otherw in wps)
                {
                    if ((!w.pos.Equals(otherw.pos) && !Physics.Linecast(w.pos, otherw.pos, LayerMask.GetMask("TransparentFX"))) || (goal_within_padding && ((w.Equals(goal) || otherw.Equals(goal)) && !Physics.Linecast(w.pos, otherw.pos, LayerMask.GetMask("Default")))))
                    {
                        w.neighbors.Add(otherw);
                        //Debug.DrawLine(w.pos, otherw.pos, Color.red, 100f);
                    }
                }
            }
            Debug.Log(String.Format("Start neighbors: {0}", start.neighbors.Count));

            // Find optimal path and draw it
            if (vehicle == "car")
            {
                optimal_path = A_star(start, goal, wps, car_g, car_h);
            } else
            {
                optimal_path = A_star(start, goal, wps, drone_g, drone_h);
            }

            Stack<Waypoint> path = new Stack<Waypoint>(new Stack<Waypoint>(optimal_path));
            
            Debug.Log(path.Count);
            Waypoint current = path.Pop();
            while (path.Count > 0)
            {
                Waypoint next = path.Pop();
                Debug.Log(current.drone_goal_vel);
                //Debug.DrawLine(current.pos, next.pos, Color.yellow, 1000f);
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
        public Stack<Waypoint> A_star(Waypoint start, Waypoint goal, List<Waypoint> waypoints, Func<Waypoint, Waypoint, Waypoint, Waypoint, List<Waypoint>, double> g, Func<Waypoint, Waypoint, double> h)
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
                    double tentative_gScore = current.gScore + g(start, goal, current, neighbor, waypoints);
                    if (current.Equals(start))
                    {
                        Debug.Log(String.Format("Neighbor coords: {0}, {1}", neighbor.pos.x, neighbor.pos.z));
                        Debug.Log(String.Format("G score: {0}", tentative_gScore));
                    }
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

        private double car_g(Waypoint start, Waypoint goal, Waypoint current, Waypoint neighbor, List<Waypoint> wps)
        {
            double g = Vector3.Distance(current.pos, neighbor.pos);
            if (current.Equals(start))
            {
                g += Vector3.Angle(Vector3.forward, neighbor.pos - current.pos);
            } else if (current.cameFrom != null)
            {
                g += Vector3.Angle(current.cameFrom.pos - current.pos, neighbor.pos - current.pos) * 0.35f;
            }
            return g;
        }

        private double drone_g(Waypoint start, Waypoint goal, Waypoint current, Waypoint neighbor, List<Waypoint> wps)
        {
            double g = Vector3.Distance(current.pos, neighbor.pos);

            if (current.cameFrom != null)
            {
                g += Vector3.Angle(current.cameFrom.pos - current.pos, neighbor.pos - current.pos) * 0.1f;
            }

            return g;
        }

        // The heuristic for the A* algorithm
        private double drone_h(Waypoint w, Waypoint goal)
        {
            return Vector3.Distance(w.pos, goal.pos);
        }

        private double car_h(Waypoint w, Waypoint goal)
        {
            return Vector3.Distance(w.pos, goal.pos);
        }

        // Create stack of waypoints and also add drone_goal_vel depending on the angle between the points
        private Stack<Waypoint> reconstruct_path(Waypoint start, Waypoint goal)
        {
            Stack<Waypoint> path = new Stack<Waypoint>();
            Waypoint current = goal;
            goal.drone_goal_vel = Vector3.forward * 15f;
            start.drone_goal_vel = Vector3.forward * 15f;

            while (current.cameFrom.cameFrom != null)
                {
                Waypoint next = current.cameFrom;
                Waypoint next_next = next.cameFrom;

                float turn_angle_ratio = Mathf.Pow(Vector3.Angle(current.pos - next.pos, next_next.pos - next.pos) / 180f, 4);
                next.drone_goal_vel = (next_next.pos - next.pos).normalized * max_turning_velocity * turn_angle_ratio;

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

        
        public Stack<Waypoint> getBezierPath()
        {
            var coarse_path = new List<Waypoint>(getOptimalPath());
            var bezier_path = new List<Waypoint>();

            float slide = 6f;

            bezier_path.Add(coarse_path[0]);
            for (int i = 1; i < coarse_path.Count - 1; i++)
            {
                Vector3 backwards_direction = (coarse_path[i - 1].pos - coarse_path[i].pos).normalized;
                Vector3 forwards_direction = (coarse_path[i + 1].pos - coarse_path[i].pos).normalized;

                Vector3 control_point_1 = coarse_path[i].pos + backwards_direction * Mathf.Min(slide, (coarse_path[i - 1].pos - coarse_path[i].pos).magnitude/2);
                Vector3 control_point_3 = coarse_path[i].pos + forwards_direction * Mathf.Min(slide, (coarse_path[i + 1].pos - coarse_path[i].pos).magnitude/2);

                for (float t = 0; t <= 1; t += 0.02f)
                {
                    Vector3 sp = BezierCurve.Quadratic(new Vector2(control_point_1.x, control_point_1.z), new Vector2(coarse_path[i].pos.x, coarse_path[i].pos.z), new Vector2(control_point_3.x, control_point_3.z), t);
                    Vector3 sample_point = new Vector3(sp.x, 0, sp.y);
                    bezier_path.Add(new Waypoint(sample_point));
                }
            }
            bezier_path.Add(coarse_path[coarse_path.Count-1]);

            for (int i = 1; i < bezier_path.Count - 1; i++)
            {
                float turn_angle_ratio = Mathf.Pow(Vector3.Angle(bezier_path[i - 1].pos - bezier_path[i].pos, bezier_path[i + 1].pos - bezier_path[i].pos) / 180f, 12);
                bezier_path[i].drone_goal_vel = (bezier_path[i + 1].pos - bezier_path[i].pos).normalized * max_turning_velocity * turn_angle_ratio;
            }
            for (int i = 0; i < bezier_path.Count - 25; i++)
            {
                if (Vector3.Distance(bezier_path[i + 25].pos, bezier_path[i].pos) < 10f && bezier_path[i + 25].drone_goal_vel.magnitude < bezier_path[i].drone_goal_vel.magnitude)
                {
                    bezier_path[i].drone_goal_vel= bezier_path[i].drone_goal_vel.normalized * Mathf.Lerp(bezier_path[i].drone_goal_vel.magnitude, bezier_path[i + 25].drone_goal_vel.magnitude, 0.8f);
                }
            }

            bezier_path[0].drone_goal_vel = (bezier_path[1].pos - bezier_path[0].pos).normalized * 15f;
            bezier_path[bezier_path.Count - 1].drone_goal_vel = (bezier_path[bezier_path.Count - 1].pos - bezier_path[bezier_path.Count - 2].pos).normalized * 15f;

            // draw goal_vels
            foreach(Waypoint w in bezier_path)
            {
                //Debug.DrawLine(w.pos, w.pos + w.drone_goal_vel, Color.yellow, 1000f);
            }

            return new Stack<Waypoint>(new Stack<Waypoint>(bezier_path));
        }
        

        public Stack<Waypoint> getSmoothPath()
        {
            var chosen_path = getOptimalPath();
            var chosen_path_list = new List<Waypoint>(new Stack<Waypoint>(getOptimalPath()));
            var coarse_path = new List<Waypoint>(chosen_path);
            List<Vector3> smooth_path;
            // linear interpolation
            int chosen_path_len = 1;
            int cps_len = 1;
            int num_interpolation;
            Waypoint current = chosen_path.Pop();
            List<Vector3> cps = new List<Vector3>(); // create contol points
            cps.Add(current.pos);
            List<Vector3> cps_new = new List<Vector3>();
            int cps_new_len = 1;
            cps_new.Add(current.pos);
            // UnityEngine.Debug.Log("cps" + current.pos);
            
            while (chosen_path.Count > 1)
            {
                Waypoint next = chosen_path.Pop();
                float linear_dis = Vector3.Distance(current.pos, next.pos);
                //UnityEngine.Debug.Log("Linear distance" + linear_dis);
                //num_interpolation = (int)Math.Ceiling(linear_dis / 2);
                num_interpolation = 11; // 
                //UnityEngine.Debug.Log("num_interpolation" + num_interpolation);
                for (int i = 1; i <= num_interpolation; i++)
                {
                    float rate = (float)i / num_interpolation;
                    Vector3 add_point = Vector3.Lerp(current.pos, next.pos, rate);
                    cps.Add(add_point);
                    //UnityEngine.Debug.Log("cps"+ add_point);
                    if (i == 2) // //
                        cps_new.Add(add_point); // //
                    if (i == 9) // //
                        cps_new.Add(add_point); // //
                }
                cps_new_len += 2;
                // cps_new.Add(next.pos);// //
                // cps_new_len++; // //
                chosen_path_len++;
                cps_len += num_interpolation;
                current = next;
            }
            cps_new.Add(current.pos);
            cps_new_len += 1;
            //UnityEngine.Debug.Log("Linear Path Length " + chosen_path_len);
            //UnityEngine.Debug.Log("Possible Control points Length " + cps_len);

            /*
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
            
            */
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

                float turn_angle_ratio = Mathf.Pow(Vector3.Angle(current_point.pos - next_point.pos, next_next.pos - next_point.pos) / 180f, 10);
                next_point.drone_goal_vel = -(next_next.pos - next_point.pos).normalized * max_turning_velocity * turn_angle_ratio;

                // TODO: Assign drone_goal_vel of closest point in coarse_path

                //next_point.drone_goal_vel = -(next_next.pos - next_point.pos).normalized * closestPoint(next_point, coarse_path).drone_goal_vel.magnitude;

                smooth_stack.Push(current_point);
                current_point = next_point;
                next_point = next_next;
                next_next = path.Pop();
            } while (path.Count > 0);
            smooth_stack.Push(next_next);

            return smooth_stack;
        }

        private Waypoint closestPoint(Waypoint point, List<Waypoint> coarse_path)
        {
            Waypoint closest = coarse_path[0];
            
            foreach (Waypoint wp in coarse_path)
            {
                if (Vector3.Distance(wp.pos, point.pos) < Vector3.Distance(closest.pos, point.pos))
                {
                    closest = wp;
                }
            }

            return closest;
        }
    }
}
