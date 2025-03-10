import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Constants
r = 50  # Cutoff distance in meters
gamma_inf = np.pi / 4  # Incidence angle
zeta = np.pi / 3  # Close-hauled angle

# Wind settings (modifiable via sliders)
default_wind_angle = np.pi / 3  # Default wind angle
wind_speed = 10  # Wind speed in knots

# Function to compute control inputs
def sailboat_controller(m, theta, psi, a, b, q):
    e = np.linalg.det(np.vstack([(b - a) / np.linalg.norm(b - a), m - a]))
    if abs(e) > r / 2:
        q = np.sign(e)
    
    phi = np.arctan2((b - a)[1], (b - a)[0])
    theta_star = phi - (2 * gamma_inf / np.pi) * np.arctan(e / r)
    
    if np.cos(psi - theta_star) + np.cos(zeta) < 0:
        theta_bar = np.pi + psi - q * zeta
    else:
        theta_bar = theta_star
    
    if np.cos(theta - theta_bar) >= 0:
        delta_r = (np.pi / 4) * np.sin(theta - theta_bar)
    else:
        delta_r = (np.pi / 4) * np.sign(np.sin(theta - theta_bar))
    
    delta_s_max = (np.pi / 2) * ((np.cos(psi - theta_bar) + 1) / 2)
    
    return delta_r, delta_s_max, q

# Visualization setup
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.set_title("Autonomous Sailboat Navigation")

# Initial boat position and path
boat_pos = np.array([0, -50])
line_start = np.array([-50, 50])
line_end = np.array([50, 50])
boat_heading = 0  # Initial heading
q_state = 1

# Draw line and boat
ax.plot([line_start[0], line_end[0]], [line_start[1], line_end[1]], 'b--', label="Target Path")
boat_marker, = ax.plot(boat_pos[0], boat_pos[1], 'ro', markersize=10, label="Boat")
wind_arrow = ax.arrow(50, 50, 20 * np.cos(default_wind_angle), 20 * np.sin(default_wind_angle), head_width=5, color='g', label="Wind")
ax.legend()

# Sliders for wind direction and speed
ax_wind_angle = plt.axes([0.15, 0.1, 0.65, 0.03])
ax_wind_speed = plt.axes([0.15, 0.05, 0.65, 0.03])
wind_angle_slider = Slider(ax_wind_angle, 'Wind Angle', 0, 2 * np.pi, valinit=default_wind_angle)
wind_speed_slider = Slider(ax_wind_speed, 'Wind Speed', 0, 20, valinit=wind_speed)

# Update function for interactivity
def update(val):
    global boat_heading, boat_pos, q_state
    wind_angle = wind_angle_slider.val
    wind_arrow.set_data(x=50, y=50)
    wind_arrow.set_dx(20 * np.cos(wind_angle))
    wind_arrow.set_dy(20 * np.sin(wind_angle))
    
    delta_r, delta_s_max, q_state = sailboat_controller(boat_pos, boat_heading, wind_angle, line_start, line_end, q_state)
    
    # Simulate movement
    boat_heading += delta_r * 0.1  # Update heading with rudder effect
    boat_pos += np.array([np.cos(boat_heading), np.sin(boat_heading)]) * 5  # Move forward
    
    boat_marker.set_data(boat_pos[0], boat_pos[1])
    fig.canvas.draw_idle()

wind_angle_slider.on_changed(update)
wind_speed_slider.on_changed(update)
plt.show()