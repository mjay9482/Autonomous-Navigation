from manim import *
import numpy as np

class ArrowSeries(Scene):
    def construct(self):
        # Parameters
        origin = ORIGIN  # Starting point of the arrows
        magnitude = 2    # Length of the arrows
        initial_theta = -90 * DEGREES  # Initial angle in radians
        end_theta = initial_theta - PI  # Final angle in radians
        num_arrows = 10   # Number of arrows to draw

        # Calculate angles for arrows
        angles = np.linspace(initial_theta, end_theta, num_arrows)

        # Draw arrows
        arrows = []
        for angle in angles:
            # Calculate direction using the angle
            direction = np.array([np.cos(angle), np.sin(angle), 0])
            # Scale the direction by the magnitude
            vector = magnitude * direction
            # Create arrow
            arrow = Arrow(start=origin, end=origin + vector, color=BLUE)
            arrows.append(arrow)

        # Add arrows to the scene
        self.play(AnimationGroup(*[Create(arrow) for arrow in arrows], lag_ratio=0.1))
        self.wait(2)

if __name__ =="__main__":
    a = ArrowSeries()
    a.render()