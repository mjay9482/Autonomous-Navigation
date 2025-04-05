from matplotlib.path import Path
from manim import *

# def hollow_rectangle(outer_vertices):
#     """
#     Creates a hollow rectangular path with an inner rectangle 2 units smaller on each side.
    
#     Parameters:
#         outer_vertices (list of tuples): List of 4 tuples specifying the vertices of the outer rectangle
#                                          in clockwise or counterclockwise order.
    
#     Returns:
#         Path: A Path object representing the hollow rectangle (outer rectangle with a hole in the center).
#     """
#     x0, y0 = outer_vertices[0]  # Bottom-left
#     x1, y1 = outer_vertices[1]  # Bottom-right
#     x2, y2 = outer_vertices[2]  # Top-right
#     x3, y3 = outer_vertices[3]  # Top-left

#     inner_vertices = [
#         (x0 + 2, y0 + 2),  # Bottom-left inner
#         (x1 - 2, y1 + 2),  # Bottom-right inner
#         (x2 - 2, y2 - 2),  # Top-right inner
#         (x3 + 2, y3 - 2),  # Top-left inner
#     ]
    
#     vertices = (
#         outer_vertices + [outer_vertices[0]]  # Close the outer rectangle
#         + inner_vertices[::-1] + [inner_vertices[0]]  # Close the inner rectangle in reverse order (hole)
#     )
    
#     # Define the codes for the path
#     codes = [Path.MOVETO] + [Path.LINETO] * (len(outer_vertices) - 1) + [Path.CLOSEPOLY] + \
#             [Path.MOVETO] + [Path.LINETO] * (len(inner_vertices) - 1) + [Path.CLOSEPOLY]

#     return Path(vertices, codes)

def cost_function1(v, vel_d):
    return np.linalg.norm(np.array(v) - np.array(vel_d))

# # def plot(V):
# #     plt.scatter(V[0,0],V[0,1], color = 'r')
# #     plt.plot(V[:,0],V[:,1], linewidth = 1)
# #     plt.xlabel('vx')
# #     plt.ylabel('vy')
# #     plt.title('Velocity profile')
# #     # plt.xlim(0.,1.2)
# #     # plt.ylim(-0.5,1.0)
# #     plt.grid(True)
# #     plt.show()

# # def plot_animation(V):
# #     fig, ax = plt.subplots()
# #     ax.set_title("Velocity Profile")
# #     ax.set_xlabel("vx")
# #     ax.set_ylabel("vy")
# #     ax.grid(True)

# #     ax.set_xlim(-2, 2)
# #     ax.set_ylim(-2, 2)

# #     scatter, = ax.plot([], [], 'ro', label="Velocity Start Point")
# #     line, = ax.plot([], [], 'b-', linewidth=1, label="Velocity Path")
# #     ax.legend()
# #     def update(frame):
# #         scatter.set_data(V[0, 0], V[0, 1])  # Start point
# #         line.set_data(V[:frame, 0], V[:frame, 1])  # Path up to the current frame
# #         return scatter, line
# #     ani = FuncAnimation(fig, update, frames=len(V), interval=100, blit=True)
# #     plt.show()

# class grad_vel(MovingCameraScene):
#     def construct(self):
#         self.camera.frame.save_state()
#         self.camera.frame.scale(1.5)
#         self.camera.frame.move_to(np.array([0, 0, 0]))

#         pA = np.array([-2, -2, 0])  
#         wpf = np.array([5, 5, 0]) 
#         vel_d = (wpf[:2] - pA[:2]) / np.linalg.norm(wpf[:2] - pA[:2])
#         steps = 0
#         V = []

#         guidance_dict = {'collision_tolerance': 0.5}
        
#         v_norm = np.linalg.norm(vel_d)
#         num_points1 = 10
#         num_points2 = 10
#         collision_radius = 2 * guidance_dict['collision_tolerance']

#         # Define outer rectangle vertices for all regions
#         vertices = np.array([[-10, -10], [10, -10], [10, 10], [-10, 10]])

#         # Create the hollow rectangular regions
#         region1 = hollow_rectangle(vertices)
#         region2 = hollow_rectangle(vertices - 2)
#         region3 = hollow_rectangle(vertices - 4)

#         # Plot the three hollow rectangular regions
#         self.play(Create(PathPatch(region1, facecolor="none", edgecolor=BLUE, lw=2)))
#         self.play(Create(PathPatch(region2, facecolor="none", edgecolor=GREEN, lw=2)))
#         self.play(Create(PathPatch(region3, facecolor="none", edgecolor=YELLOW, lw=2)))

#         agent_dot = Dot(pA, color=BLUE).scale(1.5)
#         waypoint_dot = Dot(wpf, color=GREEN).scale(1.5)
#         self.play(FadeIn(agent_dot), FadeIn(waypoint_dot))
#         invalid_vel = []
#         valid_arrows = []  
#         flag = True
#         while flag:  
#             vel_d =  (wpf[:2] - pA[:2])/np.linalg.norm(wpf[:2] - pA[:2])
#             psi = np.arctan2(vel_d[1], vel_d[0])
#             valid_arrows = []  
#             valid_velocities = []  
#             for th in np.linspace(psi , psi - np.pi, num_points1):
#                 for mag in np.linspace(0.5, v_norm + 0.5, num_points2):
#                     new_vel = np.array([mag * np.cos(th), mag * np.sin(th)])
#                     pos_f = new_vel + pA[:2]  
#                     new_pA = pA + np.array([new_vel[0], new_vel[1], 0])
                    
#                     if region0.contains_point(pos_f):
#                         new_vel = 0. * new_vel
#                     elif region1.contains_point(pos_f):
#                         new_vel = 0.25 * new_vel
#                     elif region2.contains_point(pos_f):
#                         new_vel = 0.5 * new_vel 
#                     elif region3.contains_point(pos_f):
#                         new_vel = 0.75 * new_vel
                        
#                     arrow = Arrow(
#                         start=np.array([pA[0], pA[1], 0]),  
#                         end=np.array([pos_f[0], pos_f[1], 0]),  
#                         buff=0,
#                         color=BLUE_E
#                     )
                    
#                         # valid_arrows.append(arrow)

#             # # Fade out previous arrows before creating new ones
#             # for arrow in valid_arrows:
#             #     self.play(FadeOut(arrow), run_time=0.01)

#             # for arrow in valid_arrows:  # Limiting to 10 arrows for simplicity
#             #     self.play(GrowArrow(arrow), run_time=0.01)

#             if valid_velocities:
#                 best_vel = min(valid_velocities, key=lambda v: cost_function1(v, vel_d))  # Select velocity with least deviation
#                 V.append(best_vel)
#                 new_pA = pA + np.array([best_vel[0], best_vel[1], 0])  
#                 movement_line = Line(pA, new_pA, color=YELLOW)
#                 self.play(agent_dot.animate.move_to(new_pA))  
#                 self.play(Create(movement_line))
#                 pA = new_pA

#             if np.linalg.norm(pA - wpf) < 1 or steps == 10:
#                 flag = False
#             steps += 1

#         self.wait()
#         # plot(np.array(V))
#         plot_animation(np.array(invalid_vel))

# if __name__ == "__main__":
#     HollowRectangleScene().render()

# class HollowRectangleScene(MovingCameraScene):
#     def construct(self):
#         self.camera.frame.save_state()
#         self.camera.frame.scale(4)
#         self.camera.frame.move_to(np.array([0, 0, 0]))

#         # Define the initial vertices and thickness
#         vertices = np.array([[-10, -10], [10, -10], [10, 10], [-10, 10]])
#         thickness = 2
#         num_layers = 3  # Number of concentric hollow rectangles
#         color_sequence = [BLUE, GREEN, YELLOW]  # Colors for the layers

#         # Create and display concentric hollow rectangles
#         for i in range(num_layers):
#             color = color_sequence[i % len(color_sequence)]
#             hollow_rectangle, vertices = self.create_hollow_rectangle(vertices, thickness, color=color, fill_opacity=0.3)
#             self.play(Create(hollow_rectangle))

#         # Keep the scene
#         self.wait(2)

#     def create_hollow_rectangle(self, vertices, thickness, color=BLUE, fill_opacity=0.2):
#         # Unpack the current vertices
#         x1, y1 = vertices[0]
#         x2, y2 = vertices[1]
#         x3, y3 = vertices[2]
#         x4, y4 = vertices[3]

#         # Define the outer polygon
#         outer_polygon = Polygon(
#             [x1, y1, 0], [x2, y2, 0], [x3, y3, 0], [x4, y4, 0],
#             color=color, fill_color=color, fill_opacity=fill_opacity
#         )

#         # Define the inner polygon by shrinking the current rectangle
#         inner_vertices = [
#             [x1 + thickness, y1 + thickness, 0],
#             [x2 - thickness, y2 + thickness, 0],
#             [x3 - thickness, y3 - thickness, 0],
#             [x4 + thickness, y4 - thickness, 0]
#         ]
#         inner_polygon = Polygon(*inner_vertices, color=color, fill_opacity=1.0)

#         # Combine outer and inner polygons into a hollow rectangle
#         hollow_rectangle = VGroup(outer_polygon, inner_polygon).set_fill(color, opacity=fill_opacity)
#         hollow_rectangle.set_stroke(color, width=2)

#         # Return the hollow rectangle and the new vertices for the next layer
#         new_vertices = np.array([vert[:2] for vert in inner_vertices])  # Strip Z-coordinate for next layer
#         return hollow_rectangle, new_vertices


# if __name__ == "__main__":
#     HollowRectangleScene().render()

from manim import *

class velocity_gradients(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(2.5)
        self.camera.frame.move_to(np.array([0, 0, 0]))

        pA = np.array([1, 1, 0])  
        wpf = np.array([6.5, 6.5, 0]) 
        vel_d = (wpf[:2] - pA[:2]) / np.linalg.norm(wpf[:2] - pA[:2])
        steps = 0
        V = []

        guidance_dict = {'collision_tolerance': 0.5}
        
        v_norm = np.linalg.norm(vel_d)
        num_points1 = 8
        num_points2 = 2

        # Define outer rectangle vertices for all regions
        vertices = np.array([[-12, -12], [12, -12], [12, 12], [-12, 12]])
        
        # Create the hollow rectangular regions
        region1, region1_vertices = self.create_hollow_rectangle(vertices, 3, color=BLUE, fill_opacity=0.1)
        region2, region2_vertices = self.create_hollow_rectangle(region1_vertices, 3, color=GREEN, fill_opacity=0.1)
        region3, region3_vertices = self.create_hollow_rectangle(region2_vertices, 3, color=YELLOW, fill_opacity=0.1)

        # Plot the three hollow rectangular regions
        self.play(Create(region1))
        self.play(Create(region2))
        self.play(Create(region3))

        agent_dot = Dot(pA, color=BLUE).scale(1.5)
        waypoint_dot = Dot(wpf, color=GREEN).scale(1.5)
        self.play(FadeIn(agent_dot), FadeIn(waypoint_dot))
        invalid_vel = []
        valid_arrows = []  
        flag = True

        while flag:  
            vel_d = (wpf[:2] - pA[:2]) / np.linalg.norm(wpf[:2] - pA[:2])
            psi = np.arctan2(vel_d[1], vel_d[0])
            valid_arrows = []  
            valid_velocities = []  

            for th in np.linspace(psi + np.pi, psi - np.pi, num_points1):
                # Compute initial position forward based on vel_d
                pos_f = vel_d + pA[:2]
                v = 1.
                if self.is_inside_rectangle(pA[:2], region3_vertices):
                    v *= 0.75
                elif self.is_inside_rectangle(pA[:2], region2_vertices):
                    v *= 0.5
                elif self.is_inside_rectangle(pA[:2], region1_vertices):
                    v *= 0.
                print(v)
                v_norm = np.linalg.norm(vel_d)
                # Generate new velocities after adjusting vel_d
                for mag in np.linspace(1*v, (v_norm + 1)*v, num_points2):
                    new_vel = np.array([mag * np.cos(th), mag * np.sin(th)])
                    pos_f = new_vel + pA[:2]  # New position based on this velocity

                    # Append valid velocity and create arrow for visualization
                    valid_velocities.append(new_vel)
                    arrow = Arrow(
                        start=np.array([pA[0], pA[1], 0]),  
                        end=np.array([pos_f[0], pos_f[1], 0]),  
                        buff=0,
                        color=BLUE_E
                    )
                    valid_arrows.append(arrow)

            # Visualize the arrows
            for arrow in valid_arrows:  
                self.play(GrowArrow(arrow), run_time=0.01)

            # Choose the best velocity and move the agent
            if valid_velocities:
                best_vel = min(valid_velocities, key=lambda v: cost_function1(v, vel_d))  

                V.append(best_vel)
                new_pA = pA + np.array([best_vel[0], best_vel[1], 0])  
                movement_line = Line(pA, new_pA, color=YELLOW)
                self.play(agent_dot.animate.move_to(new_pA))  
                self.play(Create(movement_line))
                pA = new_pA

            # Break the loop if the waypoint is reached
            if np.linalg.norm(pA - wpf) < 1:
                flag = False
            steps += 1

        self.wait()

    def create_hollow_rectangle(self, vertices, thickness, color=BLUE, fill_opacity=0.2):
        x1, y1 = vertices[0]
        x2, y2 = vertices[1]
        x3, y3 = vertices[2]
        x4, y4 = vertices[3]

        outer_polygon = Polygon(
            [x1, y1, 0], [x2, y2, 0], [x3, y3 , 0], [x4, y4, 0],
            color=color, fill_color=color, fill_opacity=fill_opacity
        )
        inner_vertices = [
            [x1 + thickness, y1 + thickness, 0],
            [x2 - thickness, y2 + thickness, 0],
            [x3 - thickness, y3 - thickness, 0],
            [x4 + thickness, y4 - thickness, 0]
        ]
        inner_polygon = Polygon(*inner_vertices, color=color, fill_opacity=1.0)

        hollow_rectangle = VGroup(outer_polygon, inner_polygon).set_fill(color, opacity=fill_opacity)
        hollow_rectangle.set_stroke(color, width=2)

        return hollow_rectangle, np.array([vert[:2] for vert in inner_vertices])  

    def is_inside_rectangle(self, point, vertices):
        x, y = point
        x1, y1 = vertices[0]
        x2, y2 = vertices[2]
        return x1 <= x <= x2 and y1 <= y <= y2


if __name__ == "__main__":
    velocity_gradients().render()
