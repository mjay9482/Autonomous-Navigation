from manim import *
import numpy as np

# class VelocityConeVisualization(Scene):
#     def construct(self):
#         # Apex of the velocity cone
#         apex = np.array([0, 0, 0])

#         # Obstacle relative position vector
#         dij = np.array([3, 1, 0]) 

#         # Normalize dij for direction calculation
#         dij_norm = np.linalg.norm(dij)
#         dij_p = dij / dij_norm  

#         R_tol = 1  

#         # Calculate the left ray of the cone
#         x1 = apex + dij + (2 * R_tol) * np.array([-dij_p[1], dij_p[0], 0])
#         phi1 = np.arctan2(x1[1], x1[0])
#         theta_left = phi1
#         l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left), 0])
#         left_ray = apex + l1

#         # Calculate the right ray of the cone
#         x2 = apex + dij + (2 * R_tol) * np.array([dij_p[1], -dij_p[0], 0])
#         phi2 = np.arctan2(x2[1], x2[0])
#         theta_right = phi2
#         l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right), 0])
#         right_ray = apex + l2

#         # Draw the elements on the scene
#         apex_dot = Dot(point=apex, color=BLUE)
#         dij_arrow = Arrow(apex, apex + dij, buff=0, color=GREEN)
#         left_ray_arrow = Arrow(apex, left_ray, buff=0, color=RED)
#         right_ray_arrow = Arrow(apex, right_ray, buff=0, color=RED)

#         # Add elements to the scene
#         self.add(apex_dot)
#         self.add(dij_arrow)
#         self.add(left_ray_arrow)
#         self.add(right_ray_arrow)

#         # Add labels for clarity
#         apex_label = MathTex("Apex").next_to(apex_dot, DOWN)
#         dij_label = MathTex("d_{ij}").next_to(dij_arrow, UP)
#         left_label = MathTex("Left\,Ray").next_to(left_ray_arrow, LEFT)
#         right_label = MathTex("Right\,Ray").next_to(right_ray_arrow, RIGHT)

#         self.add(apex_label, dij_label, left_label, right_label)

#         # Show the cone being drawn
#         self.play(Create(apex_dot))
#         self.play(GrowArrow(dij_arrow))
#         self.play(GrowArrow(left_ray_arrow), GrowArrow(right_ray_arrow))
#         self.wait(2)

# if __name__=="__main__":
#     vo = VelocityConeVisualization()
#     vo.render()

from manim import *

class PlotCone(VMobject):
    def __init__(self, apex, right_ray, left_ray, **kwargs):
        super().__init__(**kwargs)
        self.apex = apex
        self.right_ray = right_ray
        self.left_ray = left_ray
        self.add_cone()

    def add_cone(self):
        # Create a triangle (cone representation)
        triangle = Polygon(
            self.apex,  # Apex of the cone
            self.right_ray,  # Point on the right ray
            self.left_ray,  # Point on the left ray
            color=BLUE,
            fill_opacity=0.5,  # Slightly transparent fill
        )
        self.add(triangle)

class ConePathPlot(Scene):
    def construct(self):
        # Define points
        apex = ORIGIN
        right_ray = [2, 1, 0]  # 2 units right, 1 up
        left_ray = [2, -1, 0]  # 2 units right, 1 down

        # Add the cone to the scene
        cone = PlotCone(apex, right_ray, left_ray)
        self.add(cone)

        # Animation to highlight the cone
        self.play(FadeIn(cone))
        self.wait(2)

if __name__=="__main__":
    vo = ConePathPlot()
    vo.render()

