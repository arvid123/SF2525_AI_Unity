using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scrips
{
    class Waypoint
    {
        public Vector3 pos { get; set; }
        public List<Waypoint> neighbors { get; set; }
        public Waypoint cameFrom { get; set; }
        public double gScore { get; set; }
        public Vector3 drone_goal_vel { get; set; }

        public Waypoint(Vector3 position) 
        {
            pos = position;
            neighbors = new List<Waypoint>();
            gScore = double.PositiveInfinity;
        }
    }
}
