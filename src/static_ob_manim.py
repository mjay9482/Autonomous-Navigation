from manim import *
import numpy as np
from matplotlib.path import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def define_circular_region(c, r, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    vertices = [(r * np.cos(theta) + c[0], r * np.sin(theta) + c[1]) for theta in angles]
    vertices.append(vertices[0])
    codes = [Path.MOVETO] + [Path.LINETO] * (num_points - 1) + [Path.CLOSEPOLY]
    circular_path = Path(vertices, codes)
    return circular_path

def cost_function1(v, vel_d):
    return np.linalg.norm(np.array(v) - np.array(vel_d))

def plot(V):
    plt.scatter(V[0,0],V[0,1], color = 'r')
    plt.plot(V[:,0],V[:,1], linewidth = 1)
    plt.xlabel('vx')
    plt.ylabel('vy')
    plt.title('Velocity profile')
    # plt.xlim(0.,1.2)
    # plt.ylim(-0.5,1.0)
    plt.grid(True)
    plt.show()

def plot_animation(V):
    fig, ax = plt.subplots()
    ax.set_title("Velocity Profile")
    ax.set_xlabel("vx")
    ax.set_ylabel("vy")
    ax.grid(True)

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    scatter, = ax.plot([], [], 'ro', label="Velocity Start Point")
    line, = ax.plot([], [], 'b-', linewidth=1, label="Velocity Path")
    ax.legend()
    def update(frame):
        scatter.set_data(V[0, 0], V[0, 1])  # Start point
        line.set_data(V[:frame, 0], V[:frame, 1])  # Path up to the current frame
        return scatter, line
    ani = FuncAnimation(fig, update, frames=len(V), interval=100, blit=True)
    plt.show()

class Static_obstacle_avoidance(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(1.5)
        self.camera.frame.move_to(np.array([0, 0, 0]))

        pA = np.array([-2, -2, 0])  
        pB = np.array([2, 2, 0])  
        wpf = np.array([5, 5, 0]) 
        vel_d =  (wpf[:2] - pA[:2])/np.linalg.norm(wpf[:2] - pA[:2])
        psi = np.arctan2(vel_d[1], vel_d[0])
        steps = 0
        V = []
        
        guidance_dict = {'collision_tolerance': 0.5}
        
        v_norm = np.linalg.norm(vel_d)
        num_points1 = 10
        num_points2 = 10
        collision_radius = 2 * guidance_dict['collision_tolerance']

        circular_region = define_circular_region(pB, collision_radius)
        obstacle_circle1 = Circle(radius=collision_radius, color=RED, stroke_width=4).move_to(np.array([pB[0], pB[1], 0]))
        obstacle_dot = Dot(pB, color=RED).scale(1.5)

        num_dashes = 30  
        dashed_circle = VGroup()  

        for i in range(num_dashes):
            angle1 = 2 * np.pi * i / num_dashes
            angle2 = 2 * np.pi * (i + 1) / num_dashes
            start_point = np.array([0.5* collision_radius * np.cos(angle1), 0.5* collision_radius * np.sin(angle1), 0]) + pB
            end_point = np.array([0.5* collision_radius * np.cos(angle2), 0.5 * collision_radius * np.sin(angle2), 0]) + pB
            dash = DashedLine(start=start_point, end=end_point, color=BLUE, stroke_width=2, dash_length=0.1)
            dashed_circle.add(dash)

        self.play(FadeIn(obstacle_dot))
        self.play(Create(obstacle_circle1))  
        self.play(Create(dashed_circle))  


        agent_dot = Dot(pA, color=BLUE).scale(1.5)
        waypoint_dot = Dot(wpf, color=GREEN).scale(1.5)
        self.play(FadeIn(agent_dot), FadeIn(waypoint_dot))
        invalid_vel = []
        valid_arrows = []  
        flag = True
        while flag:  
            vel_d =  (wpf[:2] - pA[:2])/np.linalg.norm(wpf[:2] - pA[:2])
            psi = np.arctan2(vel_d[1], vel_d[0])
            valid_arrows = []  
            valid_velocities = []  
            for th in np.linspace(psi , psi - np.pi, num_points1):
                for mag in np.linspace(0.5, v_norm + 0.5, num_points2):
                    new_vel = np.array([mag * np.cos(th), mag * np.sin(th)])
                    pos_f = new_vel + pA[:2]  
                    new_pA = pA + np.array([new_vel[0], new_vel[1], 0])

                    if not circular_region.contains_point(pos_f):
                        valid_velocities.append(new_vel)
                        arrow = Arrow(
                            start=np.array([pA[0], pA[1], 0]),  
                            end=np.array([pos_f[0], pos_f[1], 0]),  
                            buff=0,
                            color=BLUE_E
                        )
                    if circular_region.contains_point(pos_f):
                        invalid_vel.append(new_vel)
                        # valid_arrows.append(arrow)

            # # Fade out previous arrows before creating new ones
            # for arrow in valid_arrows:
            #     self.play(FadeOut(arrow), run_time=0.01)

            # for arrow in valid_arrows:  # Limiting to 10 arrows for simplicity
            #     self.play(GrowArrow(arrow), run_time=0.01)

            if valid_velocities:
                best_vel = min(valid_velocities, key=lambda v: cost_function1(v, vel_d))  # Select velocity with least deviation
                V.append(best_vel)
                new_pA = pA + np.array([best_vel[0], best_vel[1], 0])  
                movement_line = Line(pA, new_pA, color=YELLOW)
                self.play(agent_dot.animate.move_to(new_pA))  
                self.play(Create(movement_line))
                pA = new_pA

            if np.linalg.norm(pA - wpf) < 1 or steps == 10:
                flag = False
            steps += 1

        self.wait()
        # plot(np.array(V))
        plot_animation(np.array(invalid_vel))
        

if __name__ == "__main__":
    Static_obstacle_avoidance().render()
