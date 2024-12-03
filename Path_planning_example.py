def dolinescross(a, b, c, d):
    M = np.array([[b[0]-a[0], c[0]-d[0]],[b[1]-a[1], c[1]-d[1]]])
    if np.linalg.det(M)==0.:
        return(False)
    v=np.array([[c[0]-a[0]],[c[1]-a[1]]])
    w = np.linalg.solve(M, v)
    if w[0]<=0:
        return(False)
    elif w[0]>=1:
        return(False)
    elif w[1]<=0:
        return(False)
    elif w[1]>=1:
        return(False)
    else:
        return(True)

from google.colab import drive
drive.mount('/content/drive')
image_path = '/content/drive/My Drive/plot_map.png'

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

m_nm=0.00053996/60

all_points_x=[]
all_points_y=[]

# Target = [9, 9]
Target = [9, 9]
all_points_x=np.append(all_points_x, Target[1])
all_points_y=np.append(all_points_y, Target[0])
Home = [1, 1]
all_points_x=np.append(all_points_x, Home[1])
all_points_y=np.append(all_points_y, Home[0])
obstacles_small = []
#obstacles_large = [[(0.10459047512914653, 36.75547052155639), (0.10354977976834119, 36.755073554622626), (0.10216576215680663, 36.75521302949116), (0.1009158547309918, 36.754236705410136), (0.09971959134486041, 36.753700263607236), (0.09886665014836615, 36.75377000104153), (0.09912414182446054, 36.75462830792535), (0.10003609151522831, 36.75452638398304), (0.10111970228710009, 36.75558317433381), (0.10277730483253583, 36.756462938890834), (0.10387164430987955, 36.75612498055533)]]
obstacles_large = [[(3, 5), (1, 5), (3, 3)], [(4.5, 2), (4.5, 5), (7.5, 5), (7.5, 2)], [(6, 6.9), (7.2, 6.5), (8, 7.5), (7.2, 8.5), (6, 8.1)]]


#shrinking obstacles for line crossing check
obstacle_centre=[0, 0]
j=-1
for obstacle in obstacles_large:
    j=j+1
    lo=len(obstacle)
    r=np.zeros(lo)
    for i in range(lo):
        r[i]=0.5*(((obstacle[i][0]-obstacle[i-1][0])**2+(obstacle[i][1]-obstacle[i-1][1])**2)**0.5)+0.5*(((obstacle[i][0]-obstacle[(i+1)%lo][0])**2+(obstacle[i][1]-obstacle[(i+1)%lo][1])**2)**0.5)
    sumx=0
    sumy=0
    lentot=0
    for i in range(lo):
        sumy=sumy+r[i]*obstacle[i][0]
        sumx=sumx+r[i]*obstacle[i][1]
        lentot=lentot+r[i]
    obstacle_centre=np.vstack((obstacle_centre, [sumx/lentot, sumy/lentot]))

obstacle_centre = np.delete(obstacle_centre, (0), axis=0)
plt.plot(obstacle_centre[:, 0], obstacle_centre[:, 1], 'r*')

def scale_coordinates(coordinates, scale_factor, center):

    #cx, cy = center
    cy, cx = center
    scaled_coords = []

    for x, y in coordinates:
        translated_x = x - cx
        translated_y = y - cy

        scaled_x = translated_x * scale_factor
        scaled_y = translated_y * scale_factor

        final_x = scaled_x + cx
        final_y = scaled_y + cy

        scaled_coords.append((final_x, final_y))
        # scaled_coords.append(final_y)

    return scaled_coords

obstacles_scaled=obstacles_large

for i in range(len(obstacles_large)):
    coords=scale_coordinates(obstacles_large[i], 0.9, obstacle_centre[i])
    for j in range(len(coords)):
        obstacles_scaled[i][j]=coords[j]


obstacles_large = [[(3, 5), (1, 5), (3, 3)], [(4.5, 2), (4.5, 5), (7.5, 5), (7.5, 2)], [(6, 6.9), (7.2, 6.5), (8, 7.5), (7.2, 8.5), (6, 8.1)]]

obstacle_lines=[0, 0, 0, 0]

def plot_image():
  ax = plt.gca()
  ax.set_xlim([0, 10])
  ax.set_ylim([0, 10])
  ax.set_aspect(aspect=1)
  # fig, ax = plt.subplots(figsize=(10, 10))
  plt.xticks([])
  plt.yticks([])



plot_image()

def generate_lines(x_coords, y_coords):

    lines = [
        [x_coords[i], y_coords[i], x_coords[i + 1], y_coords[i + 1]]
        for i in range(len(x_coords) - 1)
    ]

    return lines

scaled_lines=[0, 0, 0, 0]

for obstacle in obstacles_scaled:
    Obstacles_x=np.zeros(len(obstacle))
    Obstacles_y=np.zeros(len(obstacle))
    for point in range(len(obstacle)):
        Obstacles_x[point]=obstacle[point][1]
        Obstacles_y[point]=obstacle[point][0]

    Obstacles_x=np.append(Obstacles_x, Obstacles_x[0])
    Obstacles_y=np.append(Obstacles_y, Obstacles_y[0])

    scaled_lines=np.vstack((scaled_lines, generate_lines(Obstacles_x, Obstacles_y)))


for line in scaled_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'm--')


for obstacle in obstacles_small:
    obstacles_y=[obstacle[0]+(obstacle[2]/2+60)*m_nm, obstacle[0]-(obstacle[2]/2+60)*m_nm, obstacle[0]-(obstacle[2]/2+60)*m_nm, obstacle[0]+(obstacle[2]/2+60)*m_nm, obstacle[0]+(obstacle[2]/2+60)*m_nm]
    all_points_y=np.append(all_points_y, obstacles_y[0:4])
    obstacles_x=[obstacle[1]+(obstacle[2]/2+60)*m_nm, obstacle[1]+(obstacle[2]/2+60)*m_nm, obstacle[1]-(obstacle[2]/2+60)*m_nm, obstacle[1]-(obstacle[2]/2+60)*m_nm, obstacle[1]+(obstacle[2]/2+60)*m_nm]
    all_points_x=np.append(all_points_x, obstacles_x[0:4])
    plt.plot(obstacles_x, obstacles_y, 'r-')

    obstacle_lines=np.vstack((obstacle_lines, generate_lines(obstacles_x, obstacles_y)))

#zip


for obstacle in obstacles_large:
    Obstacles_x=np.zeros(len(obstacle))
    Obstacles_y=np.zeros(len(obstacle))
    for point in range(len(obstacle)):
        #print(obstacle[point][1])
        Obstacles_x[point]=obstacle[point][1]
        Obstacles_y[point]=obstacle[point][0]

    Obstacles_x=np.append(Obstacles_x, Obstacles_x[0])
    all_points_x=np.append(all_points_x, Obstacles_x)
    Obstacles_y=np.append(Obstacles_y, Obstacles_y[0])
    all_points_y=np.append(all_points_y, Obstacles_y)

    #plt.plot(Obstacles_x, Obstacles_y, 'r-')
    obstacle_lines=np.vstack((obstacle_lines, generate_lines(Obstacles_x, Obstacles_y)))


for line in obstacle_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'r-')

plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)

plt.show()
plot_image()

# for line in obstacle_lines:
#     plt.plot((line[0], line[2]), (line[1], line[3]), 'r-')

plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)

#print(all_points_x, all_points_y)
#plt.show()

all_points=np.array(list(zip(all_points_x, all_points_y)))

#print("all points", all_points)

all_lines=[0, 0, 0, 0]
for point1 in all_points:
    for point2 in all_points:
        plt.plot((point1[0], point2[0]), (point1[1],point2[1]), 'b-')
        all_lines=np.vstack((all_lines, [point1[0], point1[1], point2[0], point2[1]]))

#plt.plot(obstacles_scaled, marker='*', markersize=6)
for line in obstacle_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'r--')


all_lines = np.delete(all_lines, (0), axis=0)
obstacle_lines = np.delete(obstacle_lines, (0), axis=0)
scaled_lines = np.delete(scaled_lines, (0), axis=0)

#print(all_lines)

#print(len(all_lines))
#print(obstacle_lines)
i=-1
for line1 in all_lines:
    i=i+1
    #print(i)
    for line2 in obstacle_lines:
        result = dolinescross([line1[0], line1[1]], [line1[2], line1[3]],[line2[0], line2[1]],[line2[2], line2[3]])
        if result==False:
            a=1
        elif result==True:
            #print("true")
            all_lines=np.delete(all_lines, (i), axis=0)
            i=i-1
            break
        else:
            print("Error")

plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)
plt.show()

i=-1
for line1 in all_lines:
    i=i+1
    #print(i)
    for line2 in scaled_lines:
        result = dolinescross([line1[0], line1[1]], [line1[2], line1[3]],[line2[0], line2[1]],[line2[2], line2[3]])
        if result==False:
            a=1
        elif result==True:
            #print("true")
            all_lines=np.delete(all_lines, (i), axis=0)
            i=i-1
            break
        else:
            print("Error")

plot_image()

for line in all_lines:
    #print(line)
    plt.plot((line[0], line[2]), (line[1], line[3]), 'b-')

#print("length", len(all_lines))
for line in obstacle_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'r--')


plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)

plt.show()
plot_image()
#print(" ")
#print(all_lines)
#print(" ")
#print(all_points)

all_lines=all_lines.tolist()
all_points=all_points.tolist()

def dijkstra(points, paths):

    mat=np.inf*np.ones([len(points), len(points)])

    for path in paths:
        #print(path)
        #print(path[0])
        index1=points.index([path[0], path[1]])
        index2=points.index([path[2], path[3]])
        distance=((path[0]-path[2])**2+(path[1]-path[3])**2)**0.5
        mat[index1, index2]=distance

    #print(mat)

    for i in range(len(points)):
        for j in range(len(points)):
            if mat[i][j] != mat[j][i]:
                mat[i][j]=min(mat[i][j], mat[j][i])
                mat[j][i]=mat[i][j]


    n = len(mat)
    unvisited = set(range(n))
    distances = [math.inf] * n
    previous_nodes = [-1] * n

    distances[1] = 0

    while unvisited:
        current_node = min(unvisited, key=lambda node: distances[node])
        if distances[current_node] == math.inf:
            break

        for neighbor, cost in enumerate(mat[current_node]):
            if neighbor in unvisited and cost != math.inf:
                new_distance = distances[current_node] + cost
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous_nodes[neighbor] = current_node

        unvisited.remove(current_node)
        if current_node == 0:
            break

    path = []
    current = 0
    while current != -1:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()

    return distances[0], path


distance, path = dijkstra(all_points, all_lines)

print("distance", distance)
print("path", path)

pathplotx=[]
pathploty=[]
for i in path:
    pathplotx=np.append(pathplotx, all_points[i][1])
    pathploty=np.append(pathploty, all_points[i][0])

print(np.array(list(zip(pathplotx, pathploty))))


for line in obstacle_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'r-')

plt.plot(pathploty, pathplotx, 'b-')
plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)

# ax = plt.gca()
# ax.set_xlim([xmin, xmin+axdif*2])
# ax.set_ylim([ymin, ymin+axdif])
# ax.set_aspect(aspect=1)

plt.show()

plot_image()
for line in obstacle_lines:
    plt.plot((line[0], line[2]), (line[1], line[3]), 'r-')

plt.plot(Target[1], Target[0], linewidth=0, marker='*', color='magenta', markersize=5)
plt.plot(Home[1], Home[0], linewidth=0, marker='x', color='green', markersize=5)
plt.show()