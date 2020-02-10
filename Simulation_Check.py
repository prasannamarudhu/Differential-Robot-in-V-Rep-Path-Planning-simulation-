import numpy as np
import math as m
import time
import matplotlib.pyplot as plt
import vrep
import sys


#################################################################
# Pre-Allocation

PI = m.pi  # pi=3.14..., constant

vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

#vrep.simxSynchronous(clientID,1)

if clientID != -1:  # check if client connection successful
	print('Connected to remote API server')

else:
	print('Connection not successful')
	sys.exit('Could not connect')

# retrieve motor  handles
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_left_joint', vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_right_joint', vrep.simx_opmode_oneshot_wait)

errorCode, handle = vrep.simxGetObjectHandle(clientID, 'Turtlebot2', vrep.simx_opmode_oneshot_wait)
vrep.simxSetObjectPosition(clientID,handle,-1,[(0.4-5.1870), (0.4-4.8440),(0.06)],vrep.simx_opmode_oneshot_wait)
##vrep.simxSetObjectOrientation(clientID,handle,-1,[(89.7), (-62.6),(89.8)],vrep.simx_opmode_oneshot_wait)
print(handle)
emptyBuff = bytearray()
t = time.time()
################################################################


def circle(x0, y0, rad, x, y, margin):

	cir = (x - x0) ** 2 + (y - y0) ** 2 - ((margin+rad) ** 2) <= 0
	return cir


def rect(x1, x2, y1, y2, x, y, margin):

	# x1, x2 bottom left, bottom right points
	# y1, y2 top left, top right points

	# Ax + By + C = 0 is the equation of straight line
	# here A, B and C are lists of line parameters a, b and c for the 4 lines of rectangles
	A = [-1, 1, 0, 0]
	B = [0, 0, -1, 1]
	C = [x1, -x2, y1, -y2]

	rect = ((A[0] * x + B[0] * y + C[0] - margin) <= 0) and ((A[1] * x + B[1] * y + C[1] - margin) <= 0) and (
			(A[2] * x + B[2] * y + C[2] - margin) <= 0) and ((A[3] * x + B[3] * y + C[3] - margin) <= 0)

	return rect


def draw():

	# measurement in cm
	rad_bot = 17.7
	# clearance = 9.3
	clearance = 19.3
	free_space=set()
	obstacle_points = dict()
	margin = rad_bot + clearance
	# margin = 0

	for i in np.arange(0, 1110, 1):
		for j in np.arange(0, 1010, 1):

			coordinate = (i, j)
			obstacle_points[coordinate] = False
			if circle(149.95, 830.05, 79.95, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(309.73, 830.05, 79.95, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(390, 965, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(438, 736, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(438, 274, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif circle(390, 45, 40.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(149.95, 309.73, 750.1, 910, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(438, 529, 315, 498, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(529, 712, 265, 341, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(474, 748, 35, 187, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(685, 1110, 0, 35, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(927, 1110, 35, 111, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(779, 896, 35, 93, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1052, 1110, 187, 304, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(784.5, 936.5, 267, 384, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1019, 1110, 362.5, 448.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(1052, 1110, 448.5, 565.5, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(744, 1110, 621, 697, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(832, 918, 827, 1010, i, j, margin):
				obstacle_points[coordinate] = True
			elif rect(983, 1026, 919, 1010, i, j, margin):
				obstacle_points[coordinate] = True
			elif i <= margin or j <= margin or i >= 1110 - margin or j >= 1010 - margin:
				obstacle_points[coordinate] = True
			free_space.add(coordinate)

	return free_space,obstacle_points


# a class that contains all the necessary attributes that a node has while performing search
class Node:

	def __init__(self, x, y, theta, cost, id, par_id, f_cost):
		self.x = x
		self.y = y
		self.theta = theta
		self.cost = cost
		self.id = id
		self.parent_id = par_id
		self.f_cost = f_cost
		self.ur = None
		self.ul = None


def euclidean_distance(x1, y1, x2, y2):

	dist = m.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
	return dist


def generate_id(x, y):
	return y*3000 + x*800


# function to calculate the path from goal to start by retracing using parent IDs
def calc_path(closed_set, goal_node):

	# X_list and Y_list store the nodes from goal node to start node
	X_list = [goal_node.x]
	Y_list = [goal_node.y]
	p_id = goal_node.parent_id
	node_list = []
	node_list.append(goal_node)
	# retracing the path until the parent ID of the node in consideration is -1 which was the ID of source node
	while p_id != -1:
		node = closed_set[p_id]
		node_list.append(node)
		# X_list.append(node.x)
		# Y_list.append(node.y)
		p_id = node.parent_id

	return node_list


def create_neighbor(current_node, goal_node, free_space, reso, r, L,obstacle_points):

	X_old = current_node.x
	Y_old = current_node.y
	T_old = current_node.theta
	X_goal = goal_node.x
	Y_goal = goal_node.y
	cost_old = current_node.cost
	# RPM2 = 10
	# RPM1 = 5
	# delta_t = 1.25
	# 0.032
	RPM1 = 20
	delta_t = 0.038
	RPM2 = 35

	action_space = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
	neighbor_nodes = []

	for i in range(0, 8):

		left_u = (action_space[i][0])*(2*m.pi/60)
		right_u = (action_space[i][1])*(2*m.pi/60)
		x, y, t = X_old, Y_old, T_old
		for j in range(100):

			T_new = t + (right_u - left_u)*delta_t*r/L

			X_new = (x + (left_u + right_u)*m.cos(T_new)*delta_t*(r/2))
			Y_new = (y + (left_u + right_u)*m.sin(T_new)*delta_t*(r/2))
			cord = (int(X_new), int(Y_new))
			if obstacle_points[cord]:
				break
			t = T_new
			x = X_new
			y = Y_new

		if j == 99:

			X_new = int(X_new)
			Y_new = int(Y_new)
			cord = (X_new,Y_new)
			if obstacle_points[cord]:
				continue
			cost_new = cost_old + euclidean_distance(X_new, Y_new, X_old, Y_old)
			f_cost = cost_new + euclidean_distance(X_new, Y_new, X_goal, Y_goal)

			if (X_new < 1110-37 and X_new > 37) and (Y_new > 37 and Y_new < 1010-37):

				ID = generate_id(X_new, Y_new)
				neighbor_node = Node(X_new, Y_new, T_new, cost_new, ID, current_node.id, f_cost)
				# right_u = (X_new - current_node.x)/(delta_t*100)
				# left_u = (Y_new - current_node.y) / (delta_t * 100)
				neighbor_node.ur = right_u
				neighbor_node.ul = left_u

				neighbor_nodes.append(neighbor_node)

	#print("number of neighbor nodes", len(neighbor_nodes))
	return neighbor_nodes

def VRep(X,Y):

	V_left, V_right = 0,0
	X_Old = 0
	Y_Old = 0
	#RPM1=15
	#RPM2=15
	Velocity = []

	for i in range (0,len(X),1):

		x=(X[i])
		y=(Y[i])


		Len = m.sqrt(((x-X_Old)**2)+((y-Y_Old)**2))
		Len=int(Len)

		if y>Y_Old:
			for i in np.arange (0,2):
				V_left  = 0
				V_right = RPM2
				Velocity.append([V_left, V_right])

			for i in range(0, Len):
				V_left  = RPM1
				V_right = RPM2
				Velocity.append([V_left, V_right])
			X_Old = x
			Y_Old = y


		if y<Y_Old:
			for i in range(0, 2):
				V_left = RPM1
				V_right = 0
				Velocity.append([V_left, V_right])

			for i in range(0, Len):
				V_left = RPM1
				V_right = RPM2
				Velocity.append([V_left, V_right])
			X_Old = x
			Y_Old = y

	return Velocity


def isInside(circle_x, circle_y, rad, x, y):
    if ((x - circle_x) * (x - circle_x) +
            (y - circle_y) * (y - circle_y) <= rad * rad):
        return True
    else:
        return False


def a_star(start, goal, reso):

	init = time.time()

	# plotting the start and goal points
	plt.plot(start[0], start[1], "cx")
	plt.plot(goal[0], goal[1], "cx")

	free_space,obstacle_points,  = draw()
	print(time.time() - init)
	x = []
	y = []
	# for i in obstacle_points:
	# 	x.append(i[0])
	# 	y.append(i[1])
	# x = np.array(x)
	# y = np.array(y)
	points = [x for x in obstacle_points.keys() if (obstacle_points[x])]
	x = [i[0] for i in points]
	y = [i[1] for i in points]
	plt.scatter(x, y, s=1, c='b', marker='x')

	# print("execution at line 188")

	# setting the radius and distance between wheels
	r_wheel = 3.8
	L = 23

	# # checking if the start and goal points are valid
	# if start in obstacle_points or start[0] > 1110 or start[1] > 1010:
	# 	return "start point is either out of scope or inside obstacle"
	# elif goal in obstacle_points or goal[0] > 1110 or goal[1] > 1010:
	# 	return "start point is either out of scope or inside obstacle"

	# open_set is a dictionary of unexplored nodes/nodes that still need to have their cost updated to optimum value
	open_set = dict()

	# closed_set is a dictionary of explored nodes
	closed_set = dict()

	plot_points = list()

	f_cost_src = euclidean_distance(start[0], start[1], goal[0], goal[1])
	source_node = Node(start[0], start[1], 0, 0, 0, -1, f_cost_src)
	goal_node = Node(goal[0], goal[1], 0, 0, 0, 0, 0)
	source_node.ul = 0
	source_node.ur = 0
	goal_node.ul = 0
	goal_node.ur = 0

	source_node.id = generate_id(source_node.x, source_node.y)

	# storing the source node in open_set
	open_set[source_node.id] = source_node

	# continuing the search till all the nodes are explored
	while len(open_set) != 0:

		# taking the element with minimum Heuristic cost value
		id_current_node = min(open_set, key=lambda i: open_set[i].f_cost)
		current_node = open_set[id_current_node]

		# storing the explored points
		plot_points.append((current_node.x, current_node.y))
		x, y = zip(*plot_points)

		# marking the current node as explored by adding it to the closed set
		del open_set[id_current_node]
		closed_set[current_node.id] = current_node
		#print("Current",current_node.x, current_node.y)
		#print("Goal",goal_node.x, goal_node.y)
		# if int(current_node.x) in range(goal_node.x - 30, goal_node.x) and int(current_node.y) in range(goal_node.y - 30, goal_node.y):
		# init val of radius = sqrt(1100)
		if (current_node.x - goal_node.x)**2 + (current_node.y - goal_node.y)**2 <= 100:
			print("execution at line 230")
			goal_node.parent_id = current_node.parent_id
			goal_node.cost = current_node.cost
			plt.plot(x, y, "r.")
			break

		# printing the explored points for every 100/reso nodes. This helps speed up the animation.
		# CHANGE THE NUMBER FOR VARYING SPEED
		if len(plot_points) % 500 == 0:
			plt.plot(x, y, "r.")
			plt.pause(0.01)
			plot_points.clear()

		neighbor_nodes = create_neighbor(current_node, goal_node, free_space, reso, r_wheel, L,obstacle_points)

		for neighbor in neighbor_nodes:

			# ignore if already present in explored nodes
			if neighbor.id in closed_set:
				continue

			# if present in open_set then check if the cost in the current iteration is better than the previous
			if neighbor.id in open_set:
				if open_set[neighbor.id].cost > neighbor.cost:
					open_set[neighbor.id].cost = neighbor.cost
					open_set[neighbor.id].f_cost = neighbor.f_cost
					open_set[neighbor.id].parent_id = id_current_node

			else:
				open_set[neighbor.id] = neighbor

	# X_list, Y_list = calc_path(closed_set, goal_node)
	# print("X_list", X_list)
	# print("Y_list", Y_list)
	# plotting the calculated path

	node_list = calc_path(closed_set, goal_node)
	X_list = [node.x for node in node_list]
	Y_list = [node.y for node in node_list]
	plt.plot(X_list, Y_list, c='k')

	vrep_data = [[node.x, node.y, node.theta, node.ul, node.ur] for node in node_list]



	Old_X_list = 0
	Old_Y_list = 0
	Velocity_X=[]
	Velocity_Y=[]

	# X_list.reverse()
	# Y_list.reverse()
	# for i in range (0,len(X_list)):
	#
	# 	Vel_X = int(((X_list[i]) - (Old_X_list))/(2.8))
	# 	Vel_Y = int(((Y_list[i]) - (Old_Y_list))/(2.8))
	#
	# 	Velocity_X.append(Vel_X)
	# 	Velocity_Y.append(Vel_Y)
	#
	# 	Old_X_list = X_list[i]
	# 	Old_Y_list = Y_list[i]
	#
	#
	#
	# #print("Velocity_X",Velocity_X)
	# #print("Velocity_Y",Velocity_Y)

	final = time.time()
	#print("Found goal in --> seconds", final - init)
	#print("total nodes -->", len(closed_set))
	vrep_data.reverse()
	print(vrep_data)
	plt.show()

	#vrep_data = [[40, 40, 0, 0,0], [48, 44, 0.9252173913043472, 3.076, 6.153],	 [61, 67, 0.9252173913043472, 16.15, 8.461], [68, 72, 0.4626086956521697,3.84, 5.38],	 [73, 95, 0.9252173913043472, 13.84, 9.23], [89, 100, 0.9252173913043472, 11.53, 12.307],	 [120, 120, 0, 0, 0]]
	#vrep_data = [[40, 40, 0, 0,0], [44, 41, 0.4295652173913037, 0.7692307692307692, 3.0769230769230766], [47, 43, 0.8591304347826071, 1.5384615384615383, 2.3076923076923075], [48, 44, 0.6443478260869554, 0.7692307692307692, 0.7692307692307692], [51, 47, 1.0739130434782589, 2.3076923076923075, 2.3076923076923075], [54, 50, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [57, 53, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [60, 56, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [65, 61, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [66, 62, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [67, 63, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [68, 64, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [69, 65, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [70, 66, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [71, 67, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [72, 68, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [73, 69, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [74, 70, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [75, 71, 0.6443478260869554, 3.846153846153846, 3.846153846153846], [76, 72, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [77, 73, 0.8591304347826071, 3.846153846153846, 3.846153846153846], [83, 80, 0.8591304347826071, 5.384615384615384, 4.615384615384615], [89, 87, 0.8591304347826071, 5.384615384615384, 4.615384615384615], [120, 120, 0, 0,0]]
	#vrep_data = [[40, 40, 0, 0,0], [49, 57, 2.147826086956518, 13.076923076923077, 6.9230769230769225], [58, 74, 2.0816681711721685e-17, 13.076923076923077, 6.9230769230769225], [120, 120, 0, 0,0]]
	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
	for i in range(0, (len(vrep_data))):
		t = time.time()

		#x = simGetObjectMatrix(vrep_data[i+1][2])
		# res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, "remoteApiCommandServer",
		# 																			 vrep.sim_scripttype_childscript,
		# 																			 'RotMatrix', [], [(vrep_data[i+1][2])],
		# 																			 [], emptyBuff,
		# 																			 vrep.simx_opmode_blocking)

		errorcode, position=vrep.simxGetObjectPosition(clientID, 189, -1, vrep.simx_opmode_streaming)


		X_Point = (((vrep_data[i][0]) / (100)) - (5.1870))
		Y_Point = (((vrep_data[i][1]) / (100)) - (4.8440))
		#X_Point = (((vrep_data[i][0]) / (100)))
		#Y_Point = (((vrep_data[i][1]) / (100)))
		vl = vrep_data[i][3]
		vr = vrep_data[i][4]




		#while(round((position[0]),1) is not (round((((vrep_data[i + 1][0]) / (100)) - (5.1870)),1)))and (round((position[1]),1) is not (round((((vrep_data[i + 1][1]) / (100)) - (4.8440)),1))):
		#while(position[0] is not is_almost_equal(-4.7,-4.9) )and (position[1] is not is_almost_equal(-4.2,-4.5)):
		#while (int((position[0])) is not X_Point) and (int((position[1])) is not Y_Point):
		#j = 0
		# while(1):
		# 	if (isInside(X_Point,Y_Point,0.165,position[0],position[1]) == True):
		# 		vr = 0
		# 		vl = 0
		#
		# 		break

			#print ("PosX,Y",((round((position[0]),1))),(round((position[1]),1)))

			# vr=vrep_data[i+1][3]
			# vl=vrep_data[i+1][4]

			#print ("Goal Node",(round((((vrep_data[i + 1][0]) / (100)) - (5.1870)),1)),(round((vrep_data[i + 1][1]) / (100))- (4.8440)),(round((((vrep_data[i + 1][1]) / (100)) - (4.8440)),1)))


			#print ("vr,vl)",vr,vl)

		errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, vl, vrep.simx_opmode_streaming)
		errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, vr, vrep.simx_opmode_streaming)
		errorcode, position = vrep.simxGetObjectPosition(clientID, 189, -1, vrep.simx_opmode_streaming)

	# if i ==5:
	# 	exit(0)

		time.sleep(3.8)

			#print("Flag 1")
			#time.sleep(0.013) # loop executes once every 0.2 seconds (= 5 Hz)
		print ("new_position",position)
		print('Yes', X_Point, Y_Point)
		print("Velocity", vr,vl)
		print(i)
	vl = 0
	vr = 0
	errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, vl, vrep.simx_opmode_streaming)
	errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, vr, vrep.simx_opmode_streaming)



	#print("Flag 2")

	return "execution successful"


str = a_star((40, 40), (120,600), 1)
print(str)


vrep.simxFinish(clientID)
