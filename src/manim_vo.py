from manim import *
import numpy as np
# Define cost function and reachable velocities
def cost_function1(v, vel_d, goal_pos, agent_pos):
    # Use only the 2D components (x, y) for the calculation
    goal_direction = (goal_pos[:2] - agent_pos[:2]) / np.linalg.norm(goal_pos[:2] - agent_pos[:2])
    velocity_direction = v / np.linalg.norm(v)

    # Combine the cost of heading toward the goal and avoiding obstacles
    goal_direction_cost = np.linalg.norm(np.array(v) - np.array(vel_d))
    alignment_cost = np.linalg.norm(goal_direction - velocity_direction)
    return goal_direction_cost + 0.5 * alignment_cost  # Adjust weights as necessary

def reachable_velocities(vel_d, VO, obstacles, goal_pos, agent_pos):
    v_norm = np.linalg.norm(vel_d)
    num_points1 = 200  # Angular sampling
    num_points2 = 20   # Magnitude sampling
    v_new = []

    # Velocity Cone (VO)
    for th in np.linspace(-np.pi, np.pi, num_points1):
        for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
            new_vel = np.array([mag * np.cos(th), mag * np.sin(th), 0])

            v_outside = True
            for vo in VO:
                apex, left_boundary, right_boundary = vo
                # Use cross product's z-component to check relative position
                to_vel = new_vel - apex
                cross_left = np.cross(left_boundary, to_vel)[-1]  # Z-component
                cross_right = np.cross(to_vel, right_boundary)[-1]  # Z-component

                if cross_left > 0 and cross_right > 0:  # Inside the cone
                    v_outside = False
                    break
                
            for obs in obstacles:
                collision_radius = obs["radius"] + 0.1  # Add agent radius
                if np.linalg.norm(new_vel[:2] - obs["pos"][:2]) < collision_radius:
                    v_outside = False
                    break

            if v_outside:
                v_new.append(new_vel)

    v_new = np.array(v_new)
    if len(v_new) > 0:
        Vopt_A = min(v_new, key=lambda v: cost_function1(v[:2], vel_d[:2], goal_pos, agent_pos))
    else:
        Vopt_A = vel_d  # Fallback to desired velocity if no safe velocity is found

    return Vopt_A

class ReactiveGuidanceVisualization(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(1.5)  
        self.camera.frame.move_to(np.array([0, 0, 0])) 
        
        axes = Axes(
            x_range=[-4, 4],
            y_range=[-4, 4],
            axis_config={"color": WHITE},
        )
        initial_axes_labels = axes.get_axis_labels(x_label="x", y_label="y") 
        self.add(axes, initial_axes_labels)

        agent_pos = np.array([-3, -3, 0], dtype=np.float64)
        goal_pos = np.array([7, 7, 0], dtype=np.float64)
        agent_vel = np.array([1, 1, 0], dtype=np.float64)

        # Obstacles
        obstacles = [
            {"pos": np.array([0, 0, 0], dtype=np.float64), "vel": np.array([-1, -1, 0], dtype=np.float64), "radius": 0.5},
            {"pos": np.array([2, 2, 0], dtype=np.float64), "vel": np.array([-1, -1, 0], dtype=np.float64), "radius": 0.5},
        ]

        # Create agent and goal
        goal_tolerance = 0.1  # Stopping tolerance
        agent = Dot(point=agent_pos, color=BLUE, radius=0.1)
        goal = Dot(point=goal_pos, color=GREEN, radius=0.1)
        self.add(agent, goal)

        # Create obstacles
        obstacle_dots = [Dot(point=obs["pos"], color=RED, radius=0.1) for obs in obstacles]
        for dot in obstacle_dots:
            self.add(dot)

        # Velocity vector for agent
        agent_vel_arrow = Arrow(agent_pos, agent_pos + agent_vel, buff=0, color=GREEN)
        self.add(agent_vel_arrow)

        # Simulation
        for _ in range(50):
            vel_d = (goal_pos - agent_pos) / np.linalg.norm(goal_pos - agent_pos) * np.linalg.norm(agent_vel)

            # Calculate velocity cone (VO) for each obstacle
            VO = []
            for obs in obstacles:
                dij = obs["pos"] - agent_pos
                dij_norm = np.linalg.norm(dij)
                if dij_norm > 2 * obs["radius"]:
                    continue  # Skip if obstacle is too far
                
                obs_radius = obs["radius"]
                agent_radius = 0.1  
                R_tol = obs_radius + agent_radius  

                dij_unit = dij / dij_norm
                tangent_offset = R_tol / dij_norm

                left_boundary = dij_unit + np.array([-dij_unit[1], dij_unit[0], 0]) * tangent_offset
                right_boundary = dij_unit + np.array([dij_unit[1], -dij_unit[0], 0]) * tangent_offset
                VO.append((agent_pos, left_boundary, right_boundary))

            if VO:
                optimal_vel = reachable_velocities(vel_d, VO, obstacles, goal_pos, agent_pos)
            else:
                optimal_vel = vel_d
            
            if np.linalg.norm(agent_pos - goal_pos) <= goal_tolerance:
                print("Agent has reached the goal!")
                agent_vel = np.array([0., 0., 0.])
                break

            agent_vel = optimal_vel
            agent_pos += agent_vel * 0.1
            agent.move_to(agent_pos)

            for obs, dot in zip(obstacles, obstacle_dots):
                obs["pos"] += obs["vel"] * 0.1
                dot.move_to(obs["pos"])

            agent_vel_arrow.put_start_and_end_on(agent_pos, agent_pos + agent_vel)
            self.play(
                agent.animate.move_to(agent_pos),
                agent_vel_arrow.animate.put_start_and_end_on(agent_pos, agent_pos + agent_vel),
                run_time=0.1
            )


# This is the driver code to run the scene
if __name__ == "__main__":
    # Create the scene and run it
    scene = ReactiveGuidanceVisualization()
    scene.render()