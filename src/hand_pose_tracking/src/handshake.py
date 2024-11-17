import numpy as np
import matplotlib.pyplot as plt

handshake = np.array([
    [0, 0, 0],
    [0.1, 0, 0],
    [0.2, 0, 0],
    [0.3, 0.01, 0.03],
    [0.4, -0.05, -0.02],
    [0.5, -0.1, 0],
    [0.6, -0.05, 0],
    [0.7, 0.01, 0],
    [0.8, 0.05, 0],
    [0.9, 0.07, 0]
])


def interpolate_and_add_noise(handshake, points_per_segment=10, noise_level=0.01):
    """
    Interpolates between rows of the handshake array and adds random noise.
    
    Parameters:
        handshake (np.ndarray): Input array of shape (n, 3), where n is the number of key points.
        points_per_segment (int): Number of interpolated points between each step (excluding endpoints).
        noise_level (float): Maximum magnitude of random noise added to each dimension.
    
    Returns:
        np.ndarray: Interpolated and noise-augmented array of shape ((n-1)*points_per_segment + n, 3).
    """
    if handshake.shape[1] != 3:
        raise ValueError("Input 'handshake' must have 3 columns.")
    
    interpolated_points = []
    
    for i in range(len(handshake) - 1):
        start, end = handshake[i], handshake[i + 1]
        # Generate interpolated points, including start and end of the segment
        segment = np.linspace(start, end, points_per_segment, endpoint=False)
        interpolated_points.append(segment)
    
    # Append the last point of handshake
    interpolated_points.append(handshake[-1:])  # Keep shape consistent
    
    # Combine all interpolated points
    result = np.vstack(interpolated_points)
    
    # Add random noise
    noise = (np.random.rand(*result.shape) - 0.5) * 2 * noise_level  # Noise between [-noise_level, noise_level]
    result_with_noise = result + noise
    
    return result_with_noise

def compute_dominant_velocity_axis(interpolated_handshake):
    """
    Computes the axis ('X', 'Y', or 'Z') in which the velocity is greatest for each segment.

    Parameters:
        interpolated_handshake (np.ndarray): Array of shape (n, 3) representing interpolated points.

    Returns:
        list[str]: List of strings ('X', 'Y', 'Z') indicating the dominant velocity axis for each point.
    """
    # Compute velocity (difference between consecutive points)
    velocities = np.diff(interpolated_handshake, axis=0)
    
    # Identify the dominant axis for each velocity vector
    dominant_axes = []
    for velocity in velocities:
        # Find the index of the maximum absolute velocity component
        dominant_axis = np.argmax(np.abs(velocity))
        # Map index to axis name
        dominant_axes.append(['X', 'Y', 'Z'][dominant_axis])
    
    # Since the velocity is defined between points, the output list will have length (n-1)
    return dominant_axes


interpolated_handshake = interpolate_and_add_noise(handshake, points_per_segment=2, noise_level=0.01)

# Create color mapping
num_points = len(interpolated_handshake)
colors = plt.cm.viridis(np.linspace(0, 1, num_points))  # Viridis colormap

# Plot
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(projection='3d')
for i in range(num_points - 1):
    # Plot individual segments with corresponding colors
    ax.plot3D(
        interpolated_handshake[i:i+2, 0], 
        interpolated_handshake[i:i+2, 1], 
        interpolated_handshake[i:i+2, 2], 
        color=colors[i]
    )

# Add points with colors
ax.scatter3D(
    interpolated_handshake[:, 0], 
    interpolated_handshake[:, 1], 
    interpolated_handshake[:, 2], 
    c=colors, s=50
)

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
#Use the same scale for all axes
#ax.set_box_aspect([np.ptp(interpolated_handshake[:, 0]), np.ptp(interpolated_handshake[:, 1]), np.ptp(interpolated_handshake[:, 2])])
#Use range -1 to 1 for all axes
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
plt.title("Interpolated Handshake with Changing Colors")
plt.show()

# Compute dominant velocity axis
dominant_axes = compute_dominant_velocity_axis(interpolated_handshake)

# Print the result
print("Dominant velocity axes for each segment:")
print(dominant_axes)
