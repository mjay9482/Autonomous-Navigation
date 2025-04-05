from manim import *
import numpy as np

class AvoidObstacle(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(2)  
        self.camera.frame.move_to(np.array([0, 0, 0])) 
        
        axes = Axes(
            x_range=[-4, 4],
            y_range=[-4, 4],
            axis_config={"color": WHITE},
        )
        initial_axes_labels = axes.get_axis_labels(x_label="x", y_label="y") 
        self.add(axes, initial_axes_labels)

        # Define positions and parameters
        start_pos = np.array([0., 0., 0.])  # Start position in 3D
        goal_pos = np.array([8., 8., 0.])  # Goal position in 3D
        obstacle_center = np.array([4., 4., 0.])  # Obstacle center in 3D
        obstacle_radius = 1.

        # Define agent, goal, and obstacle
        agent = Dot(point=start_pos, color=BLUE)
        goal = Dot(point=goal_pos, color=GREEN)
        obstacle = Circle(radius=obstacle_radius, color=RED).move_to(obstacle_center)

        self.add(agent, goal, obstacle)

        # Velocity parameters
        agent_velocity = np.array([1., 1.])
        agent_pos = start_pos.copy()
        num_points1 = 150
        num_points2 = 15

        # Define the function to find reachable velocities
        def reachable_velocities(current_pos, current_vel, goal_pos, obstacle_center, obstacle_radius):
            v_new = []
            v_norm = np.linalg.norm(current_vel)
            psi = np.arctan2(current_vel[1], current_vel[0])

            for th in np.linspace(psi - np.pi, psi + np.pi, num_points1):
                for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
                    new_vel = np.array([mag * np.cos(th), mag * np.sin(th)])
                    future_pos = current_pos[:2] + new_vel  # Predicted position in 2D
                    # Check if the future position is outside the obstacle
                    if np.linalg.norm(future_pos - obstacle_center[:2]) > obstacle_radius:
                        v_new.append(new_vel)

            v_new = np.array(v_new)
            # Choose the velocity that minimizes the distance to the goal
            optimal_velocity = min(v_new, key=lambda v: np.linalg.norm((current_pos[:2] + v) - goal_pos[:2]))
            return optimal_velocity

        # Animation of agent's movement
        trajectory = [start_pos.copy()]
        while np.linalg.norm(agent_pos[:2] - goal_pos[:2]) > 0.1:
            # Compute optimal velocity
            agent_velocity = reachable_velocities(agent_pos, agent_velocity, goal_pos, obstacle_center, obstacle_radius)
            agent_pos[:2] += agent_velocity  # Update position in 2D
            trajectory.append(agent_pos.copy())

            # Animate agent's movement
            self.play(agent.animate.move_to(agent_pos), run_time=0.1)
            self.wait(1)

        # Draw the trajectory
        trajectory_lines = VGroup(*[Line(trajectory[i], trajectory[i + 1], color=BLUE) for i in range(len(trajectory) - 1)])
        self.play(Create(trajectory_lines))
        self.wait()


if __name__=="__main__":
    av = AvoidObstacle()
    av.render()