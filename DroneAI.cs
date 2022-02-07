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
    float goal_vel = 15f;
    Pathgen pathgen;
    float driving_time_total = 0;
    bool finished = false;
    float threshold_error = 1f;
    Rigidbody my_rigidbody;
    float k_p = 1f;
    float k_d = 1f;
    Vector3 target_position;
    Vector3 old_target_pos;
    Vector3 target_velocity;
    float time_since_target_pos;
    float allowed_error = 1f;

    private void Awake()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        pathgen = new Pathgen(terrain_manager, terrain_padding, 10f, "drone");
        chosen_path = pathgen.getBezierPath();
        var drawing_path = new Stack<Waypoint>(new Stack<Waypoint>(chosen_path));
        var current = drawing_path.Pop();
        while (drawing_path.Count > 0)
        {
            var next = drawing_path.Pop();
            Debug.DrawLine(current.pos, next.pos, Color.cyan, 10000f);
            current = next;
        }
        current_goal = chosen_path.Pop();
        my_rigidbody = GetComponent<Rigidbody>();
    }

    private Vector3 rotate_vector(Vector3 v, float radians)
    {
        return new Vector3(Mathf.Cos(radians)*v.x - Mathf.Sin(radians)*v.z, 0, Mathf.Sin(radians)*v.x + Mathf.Cos(radians)*v.z);
    }

    private void FixedUpdate()
    {
        // keep track of target position and velocity
        Vector3 dronePosition = new Vector3(transform.position.x, 0f, transform.position.z);
        Vector3 target_position = current_goal.pos;
        time_since_target_pos += Time.fixedDeltaTime;
        // Change current goal if it is reached
        if (Vector3.Distance(current_goal.pos, dronePosition) < allowed_error && chosen_path.Count > 0)
        {
            old_target_pos = target_position;
            current_goal = chosen_path.Pop();
            target_position = current_goal.pos;
            //target_velocity = (target_position - old_target_pos) / time_since_target_pos;
            target_velocity = current_goal.drone_goal_vel;
            time_since_target_pos = 0;
        }

        

        // a PD-controller to get desired velocity
        Vector3 position_error = target_position - dronePosition;
        Vector3 velocity_error = target_velocity - m_Drone.velocity;

        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

        //Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        //Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
        Debug.DrawLine(dronePosition, dronePosition + desired_acceleration, Color.red);
        //Debug.DrawLine(dronePosition, dronePosition + velocity_error, Color.red);
        //Debug.DrawLine(dronePosition, dronePosition + position_error, Color.yellow);

        // When we need to start breaking to reach the goal velocity at our current goal, start breaking
        float break_distance = Mathf.Abs((current_goal.drone_goal_vel.magnitude * current_goal.drone_goal_vel.magnitude - m_Drone.velocity.magnitude * m_Drone.velocity.magnitude) / (2 * m_Drone.acceleration.magnitude));
        if (m_Drone.velocity.magnitude > current_goal.drone_goal_vel.magnitude && break_distance >= Vector3.Distance(current_goal.pos, dronePosition) - allowed_error)
        {
            desired_acceleration = -m_Drone.velocity.normalized;
        }

        // this is how you control the car
        //if (m_Drone.velocity.magnitude > current_goal.drone_goal_vel.magnitude)
        //{
        //    m_Drone.Move(-m_Drone.velocity.normalized.x * 15f, -m_Drone.velocity.normalized.z * 15f);
        //}
        //else
        //{
        m_Drone.Move(desired_acceleration.x, desired_acceleration.z);
        //}


        /*

        // Execute your path here
        // ...
        Vector3 driving_direction = Vector3.zero;
        float gas = 1f;
        float allowed_error = 2f;
        Vector3 dronePosition = new Vector3(transform.position.x, 0f, transform.position.z);
        // Stopwatch
        if (!finished && chosen_path.Count == 0 && Vector3.Distance(current_goal.pos, dronePosition) < allowed_error)
        {
            finished = true;
            Debug.Log(string.Format("Total driving time: {0}", driving_time_total));
        } else
        {
            driving_time_total += Time.fixedDeltaTime;
        }

        // Change current goal if it is reached
        if (Vector3.Distance(current_goal.pos, dronePosition) < allowed_error && chosen_path.Count > 0)
        {
            current_goal = chosen_path.Pop();
        }

        // Default behaviour is to drive towards the goal
        driving_direction = (current_goal.pos - dronePosition).normalized;

        // When we need to start breaking to reach the goal velocity at our current goal, start breaking
        float break_distance = Mathf.Abs((current_goal.drone_goal_vel * current_goal.drone_goal_vel - m_Drone.velocity.magnitude * m_Drone.velocity.magnitude) / (2 * m_Drone.acceleration.magnitude));
        if (m_Drone.velocity.magnitude > current_goal.drone_goal_vel && break_distance >= Vector3.Distance(current_goal.pos, dronePosition) - allowed_error)
        {
            driving_direction = -m_Drone.velocity.normalized;
        }
        // Collision avoidance using sensors
        float bubble_range = 3f;
        Vector3 bounce_vec = Vector3.zero;
        float bounce_factor = 3f;
        for (int k = 0; k < 8; k++)
        {
            Vector3 ray_direction = new Vector3(Mathf.Cos(k * Mathf.PI / 4), 0, Mathf.Sin(k * Mathf.PI / 4));
            if (Physics.Raycast(transform.position, ray_direction, bubble_range)) {
                bounce_vec = (-ray_direction); 
            }
        }
        driving_direction = (driving_direction + bounce_vec * bounce_factor).normalized;
        
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

        */

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
