from manim import *

class VOVisualization(MovingCameraScene):
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

        pos_A = np.array([-3, -3, 0])  
        pos_B = np.array([3, 3, 0])  

        vel_A = np.array([2, -1, 0])   
        vel_B = np.array([1, -2, 0])   

        agent_A = Circle(radius=0.5, color=BLUE).move_to(pos_A).set_fill(BLUE, opacity=0.5)
        agent_B = Circle(radius=0.5, color=BLUE).move_to(pos_B).set_fill(BLUE, opacity=0.5)

        self.add(agent_A, agent_B)

        vel_A_arrow = Arrow(start=pos_A, end=pos_A + vel_A, color=YELLOW, buff=0)
        vel_B_arrow = Arrow(start=pos_B, end=pos_B + vel_B, color=ORANGE, buff=0)
        self.add(vel_A_arrow, vel_B_arrow)

        vel_A_label = MathTex("v_A").next_to(vel_A_arrow, UP)
        vel_B_label = MathTex("v_B").next_to(vel_B_arrow, DOWN)
        self.add(vel_A_label, vel_B_label)

        self.play(
            FadeIn(agent_A),
            FadeIn(agent_B),
            FadeIn(vel_A_arrow),
            FadeIn(vel_B_arrow),
            FadeIn(vel_A_label),
            FadeIn(vel_B_label)
        )
        self.wait(1)

        deflated_A = Circle(radius=0.01, color=BLUE).move_to(pos_A).set_fill(BLUE, opacity=0.5)
        inflated_B = Circle(radius=1, color=BLUE).move_to(pos_B).set_fill(BLUE, opacity=0.5)

        self.play(
            Transform(agent_A, deflated_A),  
            Transform(agent_B, inflated_B)   
        )

        vel_B_at_A = Arrow(start=pos_A, end=pos_A + vel_B, color=ORANGE, buff=0)
        vel_B_label_new = MathTex("v_B").next_to(vel_B_at_A, DOWN)  

        self.play(
            Transform(vel_B_arrow, vel_B_at_A),  
            Transform(vel_B_label, vel_B_label_new)  
        )
        self.wait(1)
        self.play(
            FadeOut(initial_axes_labels),
        )
        axes_labels = axes.get_axis_labels(x_label="v_x", y_label="v_y")
        self.add(axes, axes_labels)

        self.play(
            axes_labels[0].animate.set_text("v_x"),
            axes_labels[1].animate.set_text("v_y")
        )
        self.wait(1)  

        inflated_B_radius = 3.5 * inflated_B.radius
        cone_apex = pos_A

        distance_AB = np.linalg.norm(pos_B - pos_A)
        theta = np.arctan2(pos_B[1] - pos_A[1], pos_B[0] - pos_A[0])  # Angle between the centers
        beta = np.arcsin(inflated_B_radius / distance_AB)  # Half-cone angle

        tangent_point1 = pos_B + inflated_B_radius * np.array([np.cos(theta + beta), np.sin(theta + beta), 0])
        tangent_point2 = pos_B + inflated_B_radius * np.array([np.cos(theta - beta), np.sin(theta - beta), 0])

        cone_vertices = [
            cone_apex,  
            tangent_point1,  
            tangent_point2,  
        ]
        cone = Polygon(*cone_vertices, color=GREEN, fill_opacity=0.3)

        original_cone = cone.copy().set_color(BLUE)
        self.add(original_cone)

        cone_centroid = np.mean(cone_vertices, axis=0)  
        cc_label = MathTex("CC").move_to(cone_centroid + np.array([-2, -2, 0])) 
        self.add(cc_label)

        shifted_cone = cone.copy().set_color(RED)
        self.add(shifted_cone)

        self.play(
            ApplyMethod(shifted_cone.shift, vel_B),  
            run_time=2
        )

        vo_label = MathTex("VO").move_to(vel_B + np.array([2, 2, 0]))
        self.add(vo_label)

        self.wait(2)

if __name__ == "__main__":
    from manim import config
    config.media_width = "50%"
    config.verbosity = "WARNING"
    scene = VOVisualization()
    scene.render()





