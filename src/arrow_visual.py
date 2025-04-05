from manim import *

class VelocityArrows(MovingCameraScene):
    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(1)
        self.camera.frame.move_to(np.array([0, 0, 0]))

        # Parameters
        magnitude = 1.0
        psi = PI/6
        theta_start = psi           # Start angle
        theta_end =  psi -  2*PI         # End angle
        num_points = 20                 # Number of arrows
        origin = np.array([0, 0, 0])

        # Generate angles
        angles = np.linspace(theta_start, theta_end, num_points)

        # Create and animate arrows
        arrows = []
        for theta in angles:
            # Compute end point of the arrow
            end_point = np.array([
                magnitude * np.cos(theta),
                magnitude * np.sin(theta),
                0
            ])
            # Create an arrow
            arrow = Arrow(start=origin, end=end_point, color=BLUE)
            arrows.append(arrow)

        # Animate drawing of arrows
        self.play(AnimationGroup(*[GrowArrow(arrow) for arrow in arrows], lag_ratio=0.1))

        # Add grid and labels
        grid = NumberPlane(
            background_line_style={
                "stroke_color": TEAL,
                "stroke_width": 1,
                "stroke_opacity": 0.5,
            }
        )
        self.add(grid)
        self.play(Create(grid))
        self.wait(1)

        # Add title
        title = Text("Arrows Representing Velocities", font_size=24)
        title.to_edge(UP)
        self.play(Write(title))

        # Hold the final scene
        self.wait(1)

if __name__ == "__main__":
    VelocityArrows().render()