using Assets.Scrips;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Priority_Queue;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    List<Vector3> my_path;
    float terrain_padding = 4f;
    Stack<Waypoint> chosen_path;
    Waypoint current_goal;
    float goal_vel = 0;
    Pathgen pathgen;
    float driving_time_total = 0;

    private void Awake()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        pathgen = new Pathgen(terrain_manager, terrain_padding);
        chosen_path = new Stack<Waypoint>(new Stack<Waypoint>(pathgen.getOptimalPath()));
        current_goal = chosen_path.Pop();
    }


    private void FixedUpdate()
    {
        // Execute your path here
        // ...
        Vector3 driving_direction = Vector3.zero;
        float gas = 1f;
        float allowed_error = 1f;
        Vector3 dronePosition = new Vector3(transform.position.x, 0f, transform.position.z);

        // Stopwatch
        if (chosen_path.Count == 0 && Vector3.Distance(current_goal.pos, dronePosition) < allowed_error)
        {
            Debug.Log(string.Format("Total driving time: {0}", driving_time_total));
        }

        // Change current goal if it is reached
        if (Vector3.Distance(current_goal.pos, dronePosition) < allowed_error && chosen_path.Count > 0 && m_Drone.velocity.magnitude <= goal_vel + 0.1f)
        {
            current_goal = chosen_path.Pop();
        }

        // Default behaviour is to drive towards the goal
        driving_direction = (current_goal.pos - dronePosition).normalized;

        // When we need to start breaking to reach the goal velocity at our current goal, start breaking
        float break_distance = Mathf.Abs((goal_vel * goal_vel - m_Drone.velocity.magnitude * m_Drone.velocity.magnitude) / (2 * m_Drone.acceleration.magnitude));
        if (break_distance >= Vector3.Distance(current_goal.pos, dronePosition) - allowed_error)
        {
            driving_direction = -m_Drone.velocity.normalized;
        }
        // Collision avoidance using sensors
        /*float bubble_range = 6f;
        Vector3 bounce_vec = Vector3.zero;
        for (int k = 0; k < 8; k++)
        {
            Vector3 ray_direction = new Vector3(Mathf.Cos(k * Mathf.PI / 4), 0, Mathf.Sin(k * Mathf.PI / 4));
            if (Physics.Raycast(transform.position, ray_direction, bubble_range)) {
                bounce_vec = (-ray_direction); 
            }
        }*/

        driving_direction = driving_direction * gas;
        m_Drone.Move(driving_direction.x, driving_direction.z);

        // Draw line to where drone is driving
        Debug.DrawRay(transform.position, driving_direction*3f, Color.white, 1f);

        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        // this is how you control the car
        // m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
