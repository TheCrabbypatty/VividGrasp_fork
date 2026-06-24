import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots()

# Define shapes with their properties
shapes_info = [
    {"name": "Circle", "color": "blue"},
    {"name": "Rectangle", "color": "red"},
    {"name": "Triangle", "color": "green"},
]

# Create and add the circle
circle = patches.Circle((0.2, 0.5), 0.15, facecolor=shapes_info[0]["color"], edgecolor='black')
ax.add_patch(circle)

# Create and add the rectangle
rectangle = patches.Rectangle((0.45, 0.35), 0.2, 0.3, facecolor=shapes_info[1]["color"], edgecolor='black')
ax.add_patch(rectangle)

# Create and add the triangle
triangle = patches.Polygon([[0.75, 0.35], [0.95, 0.35], [0.85, 0.65]], facecolor=shapes_info[2]["color"], edgecolor='black')
ax.add_patch(triangle)

ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_aspect('equal')
plt.axis('off')
plt.show()

# Now "say" which shape has which color
for shape in shapes_info:
    print(f"The {shape['name']} is {shape['color']}.")