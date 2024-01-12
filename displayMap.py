# Massive credits to crgnam from the plotly community   


import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from PIL import Image


Re = 6356000

class displayMap():
    def __init__(self, x, u, t):
        """
        Takes as arguments:
        (1) x, the state vector's evolution over time
        (2) u, the control vector's evolution over time
        (3) t, the time

        Each of these arguments are opinionated as to order, ie. if you change any of the controls, you will have to configure the code below to match it.

        For now, the assumption is: 
        x = [position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, mass]
        u = [massflow, attitude_x, attitude_y, attitude_z]
        """

        self.x = x
        self.u = u
        self.t = t


        """
        Next, initialize the sphere object
        """
        

        def sphere(size, texture): 
            N_lat = int(texture.shape[0])
            N_lon = int(texture.shape[1])
            theta = np.linspace(0,2*np.pi,N_lat)
            phi = np.linspace(0,np.pi,N_lon)
            
            # Set up coordinates for points on the sphere
            x0 = size * np.outer(np.cos(theta),np.sin(phi))
            y0 = size * np.outer(np.sin(theta),np.sin(phi))
            z0 = size * np.outer(np.ones(N_lat),np.cos(phi))
            
            # Set up trace
            return x0,y0,z0


        original_texture = Image.open('earth.jpg')
        #resized_texture = original_texture.resize((512, 256), Image.ANTIALIAS)

        resized_texture = original_texture.resize((512, 256))
        texture = np.asarray(resized_texture).T

        colorscale =[[0.0, 'rgb(30, 59, 117)'],

                        [0.1, 'rgb(46, 68, 21)'],
                        [0.2, 'rgb(74, 96, 28)'],
                        [0.3, 'rgb(115,141,90)'],
                        [0.4, 'rgb(122, 126, 75)'],

                        [0.6, 'rgb(122, 126, 75)'],
                        [0.7, 'rgb(141,115,96)'],
                        [0.8, 'rgb(223, 197, 170)'],
                        [0.9, 'rgb(237,214,183)'],

                        [1.0, 'rgb(255, 255, 255)']]

        radius = 6356000

        x_sphere,y_sphere,z_sphere = sphere(radius,texture)
        surf = go.Surface(x=x_sphere, y=y_sphere, z=z_sphere,
                        surfacecolor=texture,
                        colorscale=colorscale, showscale=False)    

        layout = go.Layout(scene=dict(aspectratio=dict(x=1, y=1, z=1)))


        self.sphere_surf = surf
        self.golayout = layout

    


    def generate_visuals(self):
        

        x_pos = self.x[:,0]
        y_pos = self.x[:,1]
        z_pos = self.x[:,2]

        x_speed = self.x[:,3]
        y_speed = self.x[:,4]
        z_speed = self.x[:,5]

        x_att = self.u[:,1]
        y_att = self.u[:,2]
        z_att = self.u[:,3]

        massflow = self.u[:,0]

        altitude =  (np.sqrt(x_pos ** 2 + y_pos ** 2 + z_pos ** 2) - Re)
        
        speed = np.sqrt(x_speed ** 2 + y_speed ** 2 + z_speed ** 2)

        frames = [go.Frame(
            dict(
                name = f'{timestep}',
                data = [                    
                    go.Cone(
                    x=[x_pos[timestep]],
                    y=[y_pos[timestep]],
                    z=[z_pos[timestep]],
                    u=[x_att[timestep]],
                    v=[y_att[timestep]],
                    w=[z_att[timestep]],
                    sizemode="absolute",
                    sizeref=600000,
                    colorscale='jet',
                    showscale=False,
                    visible=True), # the rocket model on globe

                    go.Scatter(
                        x=[self.t[timestep]],
                        y= [massflow[timestep]],
                        marker=dict(size=15),
                        visible=True
                    ), # the dot identifying where we are on the massflow graph

                    go.Scatter(
                        x=[self.t[timestep]],
                        y= [altitude[timestep]],
                        marker=dict(size=15),
                        visible=True
                    ), # the dot identifying where we are on the altitude graph

                    go.Scatter(
                        x=[self.t[timestep]],
                        y= [speed[timestep]],
                        marker=dict(size=15),
                        visible=True
                    ), # the dot identifying where we are on the speed graph
                ],
                traces=[0,2,4,6])
            ) for timestep in range(len(self.t))]
        
        #fig = make_subplots(rows=1, cols=2, subplot_titles=("Gradient Descent", "Linear Regression"), specs=[[{'type': "scene"}, {"type": "xy"}]] )

        fig = make_subplots(rows=3, cols=2, subplot_titles=("Earth view", "Mass flow (kg/s)", "Altitude (m)", "Speed (m/s)"), specs=[ [{'type': "scene", "rowspan": 3}, {"type": "xy"}], [None, {"type": "xy"}], [None, {"type": "xy"}] ] )

        fig.add_trace(self.sphere_surf, row=1,col=1)
        fig.add_trace(self.sphere_surf, row=1,col=1) # must add it twice

        massflow_trace = go.Scatter(x=self.t, y=massflow )

        fig.add_trace(massflow_trace, row=1, col=2)
        fig.add_trace(massflow_trace, row=1, col=2) # must add it twice! remember

        altitude_trace = go.Scatter(x=self.t, y=altitude)
        fig.add_trace(altitude_trace, row=2, col=2)
        fig.add_trace(altitude_trace, row=2, col=2) # must add it twice! remember

        speed_trace = go.Scatter(x=self.t, y=speed)
        fig.add_trace(speed_trace, row=3, col=2)
        fig.add_trace(speed_trace, row=3, col=2) # must add it twice! remember

        fig.update(frames=frames)
 
        ##########################

        layout = dict(updatemenus=[
                {
                    "buttons": [{"args": [None, {"frame": {"duration": 500, "redraw": True}}],
                                "label": "Play", "method": "animate",},
                                {"args": [[None],{"frame": {"duration": 0, "redraw": False},
                                                "mode": "immediate", "transition": {"duration": 0},},],
                                "label": "Pause", "method": "animate",},],
                    "type": "buttons",
                }
            ],
            # iterate over frames to generate steps... NB frame name...
            sliders=[{"steps": [{"args": [[f.name],{"frame": {"duration": 0, 'easing': 'linear', "redraw": True},
                                                    "mode": "immediate",},],
                                "label": f.name, "method": "animate",}
                                for f in frames],}],
            height=800,
            scene=dict(
                xaxis=dict(visible=False),
                yaxis=dict(visible=False),
                zaxis=dict(visible=False),
            ),
            title_x=0.5,
            coloraxis_showscale=False,
            showlegend=False
            )
        
        fig.update_layout(
            layout
        )

        return fig
        


    


    