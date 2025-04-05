from manim import *

class GeneralizedHollowPolygon(MovingCameraScene):
    # def create_hollow_polygon(self, vertices, thickness, num_layers, color=BLUE, fill_opacity=0.2):
    #     self.camera.frame.save_state()
    #     self.camera.frame.scale(50)
    #     self.camera.frame.move_to(np.array([0., 0., 0.]))

        # def compute_inner_vertices(vertices, thickness):
        #     n = len(vertices)
        #     inner_vertices = []
        #     for i in range(n):
        #         curr = np.array(vertices[i])
        #         prev = np.array(vertices[i - 1])
        #         next = np.array(vertices[(i + 1) % n])
                
        #         edge1 = curr[:2] - prev[:2]
        #         edge2 = next[:2] - curr[:2]
                
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
                
        #         offset = normal * thickness / 2
                
        #         inner_vertex = curr[:2] + offset
        #         inner_vertices.append(np.append(inner_vertex, 0))  # Extend to 3D
        #     return inner_vertices
        
        # polygons = []
        # current_vertices = vertices
        # current_thickness = thickness

        # for _ in range(num_layers):
        #     inner_vertices = compute_inner_vertices(current_vertices, current_thickness)
        #     polygon = Polygon(*[np.array(vertex) for vertex in current_vertices], color=color, fill_opacity=fill_opacity)
        #     polygons.append(polygon)
            
        #     current_vertices = inner_vertices
        
        # hollow_polygon = VGroup(*polygons)
        # for polygon in polygons:
        #     polygon.set_fill(color, opacity=fill_opacity)
        #     polygon.set_stroke(color, width=2)

        # return hollow_polygon
                # geofence1 = np.array([[12.993496, 80.239007, 94.],
        #             [12.993500, 80.238903, 94.],
        #             [12.993485, 80.238829, 94.],
        #             [12.993359, 80.238699, 94.],
        #             [12.993547, 80.238437, 94.],
        #             [12.993716, 80.238475, 94.],
        #             [12.994077, 80.239559, 94.],
        #             [12.993809, 80.239807, 94.],
        #             [12.993271, 80.240646, 94.],
        #             [12.993050, 80.240423, 94.],
        #             [12.993144, 80.239597, 94.],
        #             [12.993565, 80.239645, 94.],
        #             [12.993570, 80.239482, 94.],
        #             [12.993193, 80.239274, 94.],
        #             [12.993044, 80.239217, 94.],
        #             [12.993054, 80.239120, 94.],
        #             [12.993335, 80.239098, 94.],
        #             [12.993496, 80.239007, 94.]])
        
    
    def create_zone(self, vertices, thickness, color=BLUE, fill_opacity=0.2):
        def compute_inner_vertices(vertices, thickness):
            n = len(vertices)
            inner_vertices = []
            for i in range(n):
                curr = np.array(vertices[i])
                prev = np.array(vertices[i - 1])
                next = np.array(vertices[(i + 1) % n])
                
                edge1 = curr[:2] - prev[:2]
                edge2 = next[:2] - curr[:2]
                
                norm1 = np.linalg.norm(edge1)
                norm2 = np.linalg.norm(edge2)
                if norm1 == 0 or norm2 == 0:
                    inner_vertices.append(curr)
                    continue
                
                edge1 /= norm1
                edge2 /= norm2
                
                bisector = edge1 + edge2
                bisector_norm = np.linalg.norm(bisector)
                if bisector_norm == 0:
                    inner_vertices.append(curr)
                    continue
                bisector /= bisector_norm
                
                normal = np.array([-bisector[1], bisector[0]])
                
                offset = normal * thickness / 2
                
                inner_vertex = curr[:2] + offset
                inner_vertices.append(np.append(inner_vertex, 0)) 
            return inner_vertices

        inner_vertices = compute_inner_vertices(vertices, thickness)
        outer_polygon = Polygon(*[np.array(vertex) for vertex in vertices], color=color, fill_opacity=fill_opacity)
        hollow_polygon = VGroup(outer_polygon)
        outer_polygon.set_fill(color, opacity=fill_opacity)
        outer_polygon.set_stroke(color, width=2)
        return hollow_polygon, inner_vertices

    def construct(self):
        self.camera.frame.save_state()
        self.camera.frame.scale(10)
        self.camera.frame.move_to(np.array([20., 0., 0.]))

        # Geofence

        geofence2 = np.array([[12.993496, 80.239007, 94.],
                            [12.993500, 80.238903, 94.],
                            [12.993485, 80.238829, 94.],
                            [12.993359, 80.238699, 94.],
                            [12.993547, 80.238437, 94.],
                            [12.993716, 80.238475, 94.],
                            [12.994077, 80.239559, 94.],
                            [12.993809, 80.239807, 94.],
                            [12.993271, 80.240646, 94.],
                            [12.993050, 80.240423, 94.],
                            [12.993144, 80.239597, 94.],
                            [12.993565, 80.239645, 94.],
                            [12.993570, 80.239482, 94.],
                            [12.993193, 80.239274, 94.],
                            [12.993044, 80.239217, 94.],
                            [12.993054, 80.239120, 94.],
                            [12.993335, 80.239098, 94.],
                            [12.993496, 80.239007, 94.]])
        sf = 1
        gps_datum = np.array([12.99300425860631, 80.23913114094384, -86.9])
        boundary_points = self.llh_to_ned(geofence2,gps_datum)
        boundary_points = np.array(boundary_points)
        boundary_points /= sf
        
        thickness = 8 
        pA = np.array([55, 15, 0])  
        wpf = np.array([55, 30, 0]) 
        vel_d = (wpf[:2] - pA[:2]) / np.linalg.norm(wpf[:2] - pA[:2])
        steps = 0
        V = []

        guidance_dict = {'collision_tolerance': 0.5}
        
        v_norm = np.linalg.norm(vel_d)
        num_points1 = 8
        num_points2 = 2

        region1, region1_vertices = self.create_zone(boundary_points, thickness, color=BLUE, fill_opacity=0.1)
        region2, region2_vertices = self.create_zone(region1_vertices, thickness, color=GREEN, fill_opacity=0.1)
        region3, region3_vertices = self.create_zone(region2_vertices, thickness, color=YELLOW, fill_opacity=0.1)
        region4, region4_vertices = self.create_zone(region3_vertices, thickness, color=YELLOW, fill_opacity=0.1)

        self.play(Create(region1))
        self.play(Create(region2))
        self.play(Create(region3))
        self.play(Create(region4))


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
                pos_f = vel_d + pA[:2]
                v = 1.
                if self.inside_zone(pA[:2], region3_vertices):
                    v *= 1.
                    new_zone = "Region 4"
                    agent_dot.set_color(ORANGE)
                elif self.inside_zone(pA[:2], region2_vertices):
                    v *= 0.75
                    new_zone = "Region 3"
                    agent_dot.set_color(GREEN)
                elif self.inside_zone(pA[:2], region1_vertices):
                    v *= 0.5
                    new_zone = "Region 2"
                    agent_dot.set_color(YELLOW)
                elif self.inside_zone(pA[:2], boundary_points):
                    v *= 0.
                    new_zone = "Region 1"
                    agent_dot.set_color(RED)
                if v == 0:
                    flag = False
                    break
                print(v)
                v_norm = np.linalg.norm(vel_d)
                for mag in np.linspace(1*v, (v_norm + 1)*v, num_points2):
                    new_vel = np.array([mag * np.cos(th), mag * np.sin(th)])
                    pos_f = new_vel + pA[:2]  

                    valid_velocities.append(new_vel)
                    arrow = Arrow(
                        start=np.array([pA[0], pA[1], 0]),  
                        end=np.array([pos_f[0], pos_f[1], 0]),  
                        buff=0,
                        color=BLUE_E
                    )
                    valid_arrows.append(arrow)

            for arrow in valid_arrows:  
                self.play(GrowArrow(arrow), run_time=0.01)

            if valid_velocities:
                best_vel = min(valid_velocities, key=lambda v: self.cost_function1(v, vel_d))  

                V.append(best_vel)
                new_pA = pA + np.array([best_vel[0], best_vel[1], 0])  
                movement_line = Line(pA, new_pA, color=YELLOW)
                self.play(agent_dot.animate.move_to(new_pA))  
                self.play(Create(movement_line))
                pA = new_pA

            if np.linalg.norm(pA - wpf) < 0.5:
                flag = False
            steps += 1

        self.wait()
        
    def inside_zone(self, point, vertices):
        x, y = point
        n = len(vertices)
        flag = False

        p1x, p1y, _ = vertices[0]
        for i in range(n + 1):
            p2x, p2y, _ = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            flag = True
            p1x, p1y = p2x, p2y

        return flag

    def cost_function1(self, v, vel_d):
        return np.linalg.norm(np.array(v) - np.array(vel_d))
  
    def llh_to_ned(self, llh, llh0):
        ans = []
        for i in range(len(llh)):
            mu0 = llh0[0] * np.pi / 180
            l0 = llh0[1] * np.pi / 180
            h0 = llh0[2]

            dllh = llh[i] - llh0
            dmu = dllh[0] * np.pi / 180
            dl = dllh[1] * np.pi / 180
            dh = dllh[2]

            re = 6378137.0
            rp = 6356752.314245
            e = np.sqrt(1 - (rp/re) ** 2)

            RN = re / np.sqrt(1 - (e * np.sin(mu0))**2)
            RM = re * (1 - e ** 2) / (1 - (e * np.sin(mu0))**2)

            xn = self.ssa(dmu) / np.arctan2(1, RN)
            yn = self.ssa(dl) / np.arctan2(1, RM * np.cos(mu0))
            zn = -dh
            ned = np.array([xn, yn, zn])
            ans.append(ned)
        return ans
    
    def ssa(self, angle):
        if angle > np.pi:
            angle = angle - 2*np.pi
        return angle
    
if __name__ == "__main__":
    GeneralizedHollowPolygon().render()
