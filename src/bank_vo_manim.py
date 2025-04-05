# from manim import *
# import numpy as np
# from matplotlib.path import Path
# from matplotlib.patches import PathPatch
# import matplotlib.pyplot as plt 

# def compute_inner_vertices(vertices, thickness):
#     n = len(vertices)
#     inner_vertices = []
#     for i in range(n):
#         curr = np.array(vertices[i])
#         prev = np.array(vertices[i - 1])
#         next = np.array(vertices[(i + 1) % n])

#         edge1 = curr - prev
#         edge2 = next - curr

#         norm1 = np.linalg.norm(edge1)
#         norm2 = np.linalg.norm(edge2)
#         if norm1 == 0 or norm2 == 0:
#             inner_vertices.append(curr)
#             continue

#         edge1 /= norm1
#         edge2 /= norm2

#         bisector = edge1 + edge2
#         bisector_norm = np.linalg.norm(bisector)
#         if bisector_norm == 0:
#             inner_vertices.append(curr)
#             continue
#         bisector /= bisector_norm

#         normal = np.array([-bisector[1], bisector[0]])

#         offset = -normal * thickness / 2

#         inner_vertex = curr + offset
#         inner_vertices.append(inner_vertex)
#     return inner_vertices

# def update_position(pos_i, goal, outer_path, inner_path):
#     vel_d = (goal[:2] - pos_i[:2]) / np.linalg.norm(goal[:2] - pos_i[:2])
#     psi = np.arctan2(vel_d[1], vel_d[0])

#     valid_velocities = []  
#     v = 1.0
#     num_points1 = 5
#     num_points2 = 5
#     v_norm = np.linalg.norm(vel_d)
#     # for theta in np.linspace(psi + np.pi, psi - np.pi, num_points1):
#     #     for mag in np.linspace(v_norm, v_norm + 0.5, num_points2):
#     #         x_component = mag * np.cos(theta)
#     #         y_component = mag * np.sin(theta)
#     #         new_vel = np.array([x_component, y_component])
              
#     #         pos_f = pos_i[:2] + new_vel
#     #         if inner_path.contains_point(pos_f) and outer_path.contains_point(pos_f):
#     #             valid_velocities.append(new_vel)
#     for theta in np.linspace(psi + np.pi, psi - np.pi, num_points1):
#         for mag in np.linspace(v_norm, v_norm + 0.5, num_points2):
#             x_component = mag * np.cos(theta)
#             y_component = mag * np.sin(theta)
#             new_vel = np.array([x_component, y_component])  
#             pos_f = pos_i[:2] + new_vel*5  # Position after applying velocity
            
#             # Check x-coordinate validity
#             if inner_path.contains_point([pos_f[0], pos_i[1]]) and outer_path.contains_point([pos_f[0], pos_i[1]]):
#                 valid_x = True
#             else:
#                 valid_x = False

#             # Check y-coordinate validity
#             if inner_path.contains_point([pos_i[0], pos_f[1]]) and outer_path.contains_point([pos_i[0], pos_f[1]]):
#                 valid_y = True
#             else:
#                 valid_y = False

#             if valid_x and valid_y:
#                 valid_velocities.append(new_vel)
#             else:
#                 if not valid_x:
#                     new_vel[0] = 0  
#                 if not valid_y:

#                     new_vel[1] = 0  

#                 if inner_path.contains_point([pos_i[0] + new_vel[0], pos_i[1] + new_vel[1]]) and \
#                     outer_path.contains_point([pos_i[0] + new_vel[0], pos_i[1] + new_vel[1]]):
#                     valid_velocities.append(new_vel)

#     if not valid_velocities:
#         valid_velocities.append(np.array([0, 0]))

#     if valid_velocities:
#         best_vel = min(valid_velocities, key=lambda v: np.linalg.norm(v - vel_d))
#         pos_i = pos_i.astype(np.float64)
#         pos_i += best_vel 

#     if np.linalg.norm(pos_i - goal) < 1:
#         return pos_i, True

#     return pos_i, False

# def boundary_to_polygon(boundary_points):
#     return [np.array([x, y, 0]) for x, y in boundary_points]

# def rectangular_patch(vertices):
#     codes = [
#         Path.MOVETO,    
#         Path.LINETO,    
#         Path.LINETO,    
#         Path.LINETO,    
#         Path.CLOSEPOLY  
#     ]
#     rectangle_path = Path(vertices, codes)
#     return rectangle_path

# def visualize_rectangle(vertices, color=BLUE, stroke_width=3):
#     polygon = Polygon(
#         *[np.array([x, y, 0]) for x, y in vertices],
#         color=color,
#         stroke_width=stroke_width
#     )
#     return polygon

# class TrajectoryAnimation(MovingCameraScene):
#     def construct(self):
#         self.camera.frame.save_state()
#         self.camera.frame.scale(4)
#         self.camera.frame.move_to(np.array([0., -2., 0.]))
 
#         boundary_points = np.array([
#             [-8., 8.], [-4., 8.], [0., 8.], [4., 8.], [8., 8.],
#             [8., 6.], [8., 4.], [10., 4.], [12., 4.], [14., 4.],
#             [16., 4.], [16., 2.], [16., 0.], [16., -2.], [16., -4.],
#             [14., -4.], [12., -4.], [10., -4.], [8., -4.], [8., -6.],
#             [8., -8.], [8., -14.], [4., -14.], [0., -14.], [-4., -14.], [-8., -14.],
#             [-8., -8.], [-8., -6.], [-8., -2.], [-8., 0.], [-8., 2.],
#             [-8., 4.], [-8., 6.]
#         ])
        
#         # Rectangles
#         bank1 = [[8, 4], [8, 8], [16, 8], [16, 4], [8, 4]]
#         bank2 = [[8, -14], [8, -4], [16, -4], [16, -14], [8, -14]]
#         bank3 = [[-12, -14], [-12, 8], [-8, 8], [-8, -14], [-12, -14]]
#         bank4 = [[-12, -14], [-12, 8], [16, 8], [16, -14], [-12, -14]]

#         thickness = 4.5
#         inner_boundary_points = np.array(compute_inner_vertices(boundary_points, thickness))

#         outer_polygon = Polygon(*boundary_to_polygon(boundary_points), color=RED, stroke_width=4)
#         inner_polygon = Polygon(*boundary_to_polygon(inner_boundary_points), color=BLUE, stroke_width=2)
#         # self.add(outer_polygon, inner_polygon)

#         rect1 = visualize_rectangle(bank1, color=GREEN)
#         rect2 = visualize_rectangle(bank2, color=YELLOW)
#         rect3 = visualize_rectangle(bank3, color=ORANGE)
#         rect4 = visualize_rectangle(bank4, color=RED)
#         self.add(rect1, rect2, rect3, rect4)

#         pos_i = np.array([0, -10])
#         goal = np.array([13, 0])
#         agent_dot = Dot(point=np.array([pos_i[0], pos_i[1], 0]), color=YELLOW)
#         waypoint_dot = Dot(point=np.array([goal[0], goal[1], 0]), color=GREEN)
#         self.add(agent_dot, waypoint_dot)

#         codes_outer = [Path.MOVETO] + [Path.LINETO] * (len(boundary_points[:, :2]) - 1) + [Path.CLOSEPOLY]
#         boundary_points_2d = np.vstack([boundary_points[:, :2], boundary_points[0, :2]])
#         outer_path = Path(boundary_points_2d, codes_outer)
        
#         codes_inner = [Path.MOVETO] + [Path.LINETO] * (len(inner_boundary_points[:, :2]) - 1) + [Path.CLOSEPOLY]
#         inner_boundary_points_2d = np.vstack([inner_boundary_points[:, :2], inner_boundary_points[0, :2]])
#         inner_path = Path(inner_boundary_points_2d, codes_inner)

#         trajectory = [pos_i.copy()]
#         for _ in range(200):  
#             pos_i, goal_reached = update_position(pos_i, goal, outer_path, inner_path)
#             trajectory.append(pos_i.copy())
#             new_dot = Dot(point=np.array([pos_i[0], pos_i[1], 0]), color=YELLOW)
#             self.play(Transform(agent_dot, new_dot), run_time=0.1)
#             if goal_reached:
#                 break

#         self.wait(3)

# if __name__ == "__main__":
#     TrajectoryAnimation().render()



# from shapely.geometry import Polygon
# from shapely.geometry.polygon import orient
# import matplotlib.pyplot as plt
# import numpy as np

# def pad_boundary(vertices, padding_distance):
#     # Create a polygon from vertices
#     polygon = Polygon(vertices)
#     polygon = orient(polygon)  # Ensure correct orientation

#     # Pad the boundary
#     padded_polygon = polygon.buffer(padding_distance)

#     # Extract the padded vertices
#     padded_vertices = list(padded_polygon.exterior.coords)
#     return padded_vertices

# # Example: Original boundary and padding
# # original_boundary = [(0, 0), (4, 0), (4, 4), (0, 4), (0, 0)]  
# original_boundary = np.array([
#             [-8., 8.], [-4., 8.], [0., 8.], [4., 8.], [8., 8.],
#             [8., 6.], [8., 4.], [10., 4.], [12., 4.], [14., 4.],
#             [16., 4.], [16., 2.], [16., 0.], [16., -2.], [16., -4.],
#             [14., -4.], [12., -4.], [10., -4.], [8., -4.], [8., -6.],
#             [8., -8.], [8., -14.], [4., -14.], [0., -14.], [-4., -14.], [-8., -14.],
#             [-8., -8.], [-8., -6.], [-8., -2.], [-8., 0.], [-8., 2.],
#             [-8., 4.], [-8., 6.]
#         ])
# padding_distance = -1.0  # positive for Outward padding

# # Pad the boundary
# padded_boundary = pad_boundary(original_boundary, padding_distance)

# # Visualization
# fig, ax = plt.subplots()
# original_polygon = Polygon(original_boundary)
# padded_polygon = Polygon(padded_boundary)

# # Plot original boundary
# x, y = original_polygon.exterior.xy
# ax.plot(x, y, label="Original Boundary", color="blue")

# # Plot padded boundary
# x, y = padded_polygon.exterior.xy
# ax.plot(x, y, label="Padded Boundary", color="red")

# ax.set_aspect('equal', 'box')
# plt.legend()
# plt.show()


from manim import *
import numpy as np
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LinearRing
from matplotlib.path import Path
from matplotlib.patches import PathPatch

def boundary_to_polygon(boundary_points):
    return [np.array([x, y, 0]) for x, y in boundary_points]

def create_padded_boundaries(boundary_points, thickness):
    original_polygon = ShapelyPolygon(boundary_points)
    outer_polygon = original_polygon.buffer(thickness)
    inner_polygon = original_polygon.buffer(-thickness)

    outer_boundary = list(LinearRing(outer_polygon.exterior).coords)
    inner_boundary = list(LinearRing(inner_polygon.exterior).coords)

    return np.array(outer_boundary), np.array(inner_boundary)


def update_position(pos_i, goal, outer_path, inner_path):
    vel_d = (goal[:2] - pos_i[:2]) / np.linalg.norm(goal[:2] - pos_i[:2])
    psi = np.arctan2(vel_d[1], vel_d[0])

    valid_velocities = []  
    v = 1.0
    num_points1 = 5
    num_points2 = 5
    v_norm = np.linalg.norm(vel_d)
    # for theta in np.linspace(psi + np.pi, psi - np.pi, num_points1):
    #     for mag in np.linspace(v_norm, v_norm + 0.5, num_points2):
    #         x_component = mag * np.cos(theta)
    #         y_component = mag * np.sin(theta)
    #         new_vel = np.array([x_component, y_component])
              
    #         pos_f = pos_i[:2] + new_vel
    #         if inner_path.contains_point(pos_f) and outer_path.contains_point(pos_f):
    #             valid_velocities.append(new_vel)
    for theta in np.linspace(psi + np.pi, psi - np.pi, num_points1):
        for mag in np.linspace(v_norm, v_norm + 0.5, num_points2):
            x_component = mag * np.cos(theta)
            y_component = mag * np.sin(theta)
            new_vel = np.array([x_component, y_component])  
            pos_f = pos_i[:2] + new_vel*5  # Position after applying velocity
            
            # Check x-coordinate validity
            if inner_path.contains_point([pos_f[0], pos_i[1]]) and outer_path.contains_point([pos_f[0], pos_i[1]]):
                valid_x = True
            else:
                valid_x = False

            # Check y-coordinate validity
            if inner_path.contains_point([pos_i[0], pos_f[1]]) and outer_path.contains_point([pos_i[0], pos_f[1]]):
                valid_y = True
            else:
                valid_y = False

            if valid_x and valid_y:
                valid_velocities.append(new_vel)
            else:
                if not valid_x:
                    new_vel[0] = 0  
                if not valid_y:

                    new_vel[1] = 0  

                if inner_path.contains_point([pos_i[0] + new_vel[0], pos_i[1] + new_vel[1]]) and \
                    outer_path.contains_point([pos_i[0] + new_vel[0], pos_i[1] + new_vel[1]]):
                    valid_velocities.append(new_vel)

    if not valid_velocities:
        valid_velocities.append(np.array([0, 0]))

    if valid_velocities:
        best_vel = min(valid_velocities, key=lambda v: np.linalg.norm(v - vel_d))
        pos_i = pos_i.astype(np.float64)
        pos_i += best_vel 

    if np.linalg.norm(pos_i - goal) < 1:
        return pos_i, True

    return pos_i, False

def rectangular_patch(vertices):
    codes = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY
    ]
    rectangle_path = Path(vertices, codes)
    return rectangle_path

def visualize_rectangle(vertices, color=BLUE, stroke_width=3):
    polygon = Polygon(
        *[np.array([x, y, 0]) for x, y in vertices],
        color=color,
        stroke_width=stroke_width
    )
    return polygon

class TrajectoryAnimation(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(4)
        self.camera.frame.move_to(np.array([0., -2., 0.]))
 
        boundary_points = np.array([
            [-8., 8.], [-4., 8.], [0., 8.], [4., 8.], [8., 8.],
            [8., 6.], [8., 4.], [10., 4.], [12., 4.], [14., 4.],
            [16., 4.], [16., 2.], [16., 0.], [16., -2.], [16., -4.],
            [14., -4.], [12., -4.], [10., -4.], [8., -4.], [8., -6.],
            [8., -8.], [8., -14.], [4., -14.], [0., -14.], [-4., -14.], [-8., -14.],
            [-8., -8.], [-8., -6.], [-8., -2.], [-8., 0.], [-8., 2.],
            [-8., 4.], [-8., 6.]
        ])
        
        # Rectangles
        bank1 = [[8, 4], [8, 8], [16, 8], [16, 4], [8, 4]]
        bank2 = [[8, -14], [8, -4], [16, -4], [16, -14], [8, -14]]
        bank3 = [[-12, -14], [-12, 8], [-8, 8], [-8, -14], [-12, -14]]
        bank4 = [[-12, -14], [-12, 8], [16, 8], [16, -14], [-12, -14]]

        thickness = -1
        outer_boundary_points, inner_boundary_points = create_padded_boundaries(boundary_points, thickness)

        outer_polygon = Polygon(*boundary_to_polygon(outer_boundary_points), color=RED, stroke_width=4)
        inner_polygon = Polygon(*boundary_to_polygon(inner_boundary_points), color=BLUE, stroke_width=2)
        self.add(outer_polygon, inner_polygon)

        rect1 = visualize_rectangle(bank1, color=GREEN)
        rect2 = visualize_rectangle(bank2, color=YELLOW)
        rect3 = visualize_rectangle(bank3, color=ORANGE)
        rect4 = visualize_rectangle(bank4, color=RED)
        # self.add(rect1, rect2, rect3, rect4)

        pos_i = np.array([0, -10])
        goal = np.array([13, 0])
        agent_dot = Dot(point=np.array([pos_i[0], pos_i[1], 0]), color=YELLOW)
        waypoint_dot = Dot(point=np.array([goal[0], goal[1], 0]), color=GREEN)
        self.add(agent_dot, waypoint_dot)
        codes_outer = [Path.MOVETO] + [Path.LINETO] * (len(outer_boundary_points) - 2) + [Path.CLOSEPOLY]
        codes_inner = [Path.MOVETO] + [Path.LINETO] * (len(inner_boundary_points) - 2) + [Path.CLOSEPOLY]

        outer_path = Path(outer_boundary_points, codes_outer)
        inner_path = Path(inner_boundary_points, codes_inner)

        trajectory = [pos_i.copy()]
        for _ in range(200):  
            pos_i, goal_reached = update_position(pos_i, goal, outer_path, inner_path)
            trajectory.append(pos_i.copy())
            new_dot = Dot(point=np.array([pos_i[0], pos_i[1], 0]), color=YELLOW)
            self.play(Transform(agent_dot, new_dot), run_time=0.1)
            if goal_reached:
                break

        self.wait(3)

if __name__ == "__main__":
    TrajectoryAnimation().render()
