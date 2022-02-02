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

        public Pathgen(TerrainManager t, float tp)
        {
            terrain_manager = t;
            terrain_padding = tp;

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
                    RaycastHit hit;
                    if ((!Physics.Linecast(w.pos, otherw.pos) && !w.pos.Equals(otherw.pos)) || ((w.Equals(goal) || otherw.Equals(goal) || w.Equals(start) || otherw.Equals(start)) && !Physics.Linecast(w.pos, otherw.pos, LayerMask.GetMask("Default"))))
                    {
                        w.neighbors.Add(otherw);
                        Debug.DrawLine(w.pos, otherw.pos, Color.red, 100f);
                    }
                }
            }

            // Find optimal path and draw it
            optimal_path = A_star(start, goal, wps);
            
            Debug.Log(optimal_path.Count);
            Waypoint current = optimal_path.Pop();
            while (optimal_path.Count > 0)
            {
                Waypoint next = optimal_path.Pop();
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
            return optimal_path;
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


    }
}
