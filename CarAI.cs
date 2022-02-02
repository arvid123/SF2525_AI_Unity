using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Assets.Scrips;
using Priority_Queue;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends; // use these to avoid collisions

        public GameObject my_goal_object;
        List<Vector3> my_path;
        float terrain_padding = 4f;

        private void Start()
        {
            // get the drone controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


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
            while (optimal_path.Count > 0)
            {
                Waypoint next = optimal_path.Pop();
                Debug.DrawLine(current.pos, next.pos, Color.yellow, 1000f);
                current = next;
            }
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


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            /*Vector3 relVect = my_goal_object.transform.position - transform.position;
            bool is_in_front = Vector3.Dot(transform.forward, relVect) > 0f;
            bool is_to_right = Vector3.Dot(transform.right, relVect) > 0f;

            if(is_in_front && is_to_right)
                m_Car.Move(1f, 1f, 0f, 0f);
            if(is_in_front && !is_to_right)
                m_Car.Move(-1f, 1f, 0f, 0f);
            if(!is_in_front && is_to_right)
                m_Car.Move(-1f, -1f, -1f, 0f);
            if(!is_in_front && !is_to_right)
                m_Car.Move(1f, -1f, -1f, 0f);*/


            // this is how you control the car
            //m_Car.Move(1f, 1f, 1f, 0f);

        }
    }
}
