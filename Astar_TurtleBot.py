import numpy as np
import math as m
import time
import matplotlib.pyplot as plt
import vrep
import sys


#################################################################
# V-Rep Remote API Server Connection
e
PI = m.pi  # pi=3.14..., constant

vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
#vrep.simxSynchronous(clientID,1)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

if clientID != -1:  # check if client connection successful
	print('Connected to remote API server')

else:
	print('Connection not successful')
	sys.exit('Could not connect')

# Obtaining the Turtlebot left and right wheel handle ID
errorCode, left_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_left_joint', vrep.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = vrep.simxGetObjectHandle(clientID, 'wheel_right_joint', vrep.simx_opmode_oneshot_wait)

errorCode, handle = vrep.simxGetObjectHandle(clientID, 'Turtlebot2', vrep.simx_opmode_oneshot_wait)
vrep.simxSetObjectPosition(clientID,handle,-1,[(0.4-5.1870), (0.4-4.8440),(0.06)],vrep.simx_opmode_oneshot_wait)
t = time.time()
################################################################

# Radius of wheel and the distance between wheels are assigned
Rad_Wheel = 3.8
WheelBase = 23
#The Radius of the bot and additional clearance are assigned to obtain the final clearance values
Bot_Radius = 17.7
Bot_clearance = 12.3
Fin_Clearance = Bot_Radius + Bot_clearance
#This function will create a map with obstacle points for the robot to traverse
def Obstacle_mapping(start,goal):

	# Rectangle:
	A1 = [-1, 1, 0, 0]
	B1 = [0, 0, -1, 1]
	C1 = [149.95, -309.73, 750.1, -910]

	A2 = [-1, 1, 0, 0]
	B2 = [0, 0, -1, 1]
	C2 = [438, -529, 315, -498]

	A3 = [-1, 1, 0, 0]
	B3 = [0, 0, -1, 1]
	C3 = [529, -712, 265, -341]

	A4 = [-1, 1, 0, 0]
	B4 = [0, 0, -1, 1]
	C4 = [474, -748, 35, -187]

	A5 = [-1, 1, 0, 0]
	B5 = [0, 0, -1, 1]
	C5 = [685, -1110, 0, -35]

	A6 = [-1, 1, 0, 0]
	B6 = [0, 0, -1, 1]
	C6 = [927, -1110, 35, -111]

	A7 = [-1, 1, 0, 0]
	B7 = [0, 0, -1, 1]
	C7 = [779, -896, 35, -93]

	A8 = [-1, 1, 0, 0]
	B8 = [0, 0, -1, 1]
	C8 = [1052, -1110, 187, -304]


	A9 = [-1, 1, 0, 0]
	B9 = [0, 0, -1, 1]
	C9 = [784.5, -936.5, 267, -384]

	A10 = [-1, 1, 0, 0]
	B10 = [0, 0, -1, 1]
	C10 = [1019, -1110, 362.5, -448.5]

	Fin_Clearance = Bot_Radius + Bot_clearance
	A11 = [-1, 1, 0, 0]
	B11 = [0, 0, -1, 1]
	C11 = [1052, -1110, 448.5, -565.5]


	A12 = [-1, 1, 0, 0]
	B12 = [0, 0, -1, 1]
	C12 = [744, -1110, 621, -697]

	A13 = [-1, 1, 0, 0]
	B13 = [0, 0, -1, 1]
	C13 = [832, -918, 827, -1010]
	Points_MapObstacle = dict()

	A14 = [-1, 1, 0, 0]
	B14 = [0, 0, -1, 1]
	C14 = [983, -1026, 919, -1010]


	# Circle.
	r1 = 79.95
	r2 = 79.95
	r = 40.5

	#Drawing the Obstacle Map points in the graph
	for i in np.arange(0, 1110, 1):
		for j in np.arange(0, 1010, 1):
			#Defining the axis points
			Axis_Points = (i, j)
			#Initializing the Obstacle space points array
			Points_MapObstacle[Axis_Points] = False
			if drawcircle(149.95, 830.05, r1, i, j, Fin_Clearance): Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(438, 529, 315, 498, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(529, 712, 265, 341, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True

			#The drawcircle function draws the obstacle in circle shape and the drawrectangle space draws it in rectangle shape
			elif draw_rectangle_obst(779, 896, 35, 93, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif drawcircle(309.73, 830.05, r1, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif drawcircle(390, 965, r, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif drawcircle(438, 736, r, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(744, 1110, 621, 697, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(832, 918, 827, 1010, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(983, 1026, 919, 1010, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif drawcircle(438, 274, r, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif drawcircle(390, 450, r, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(474, 748, 35, 187, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(685, 1110, 0, 35, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(927, 1110, 35, 111, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(1052, 1110, 187, 304, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(784.5, 936.5, 267, 384, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(1019, 1110, 362.5, 448.5, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(1052, 1110, 448.5, 565.5, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			elif draw_rectangle_obst(149.95, 309.73, 750.1, 910, i, j, Fin_Clearance):Points_MapObstacle[Axis_Points] = True
			#The corner clearance for the robot is defined below
			elif i <= Fin_Clearance or j <= Fin_Clearance or i >= 1110 - Fin_Clearance or j >= 1010 - Fin_Clearance:Points_MapObstacle[Axis_Points] = True

	return Points_MapObstacle

#	This function accepts the input fron the user
def user_input():
    print("Please feed the Following Information for Path Planning ")
    sx = input("Enter the Start node 'X' value in Centimeter: ")
    sy = input("Enter the Start node 'Y' value in Centimeter: ")
    gx = input("Enter the Goal node 'X' value in Centimeter: ")
    gy = input("Enter the Goal node 'Y' value in Centimeter: ")

    return float(sx), float(sy), float(gx), float(gy)
    # return sx, sy, rrad, clrnc, resol, gx, gy


#################
#	The below mentioned class defines all the variables and objects used inside this algorithm
class Algorithm_PathCalcNode:
	def __init__(self, x, y, Angle, Val_Cost, id, Parent_ID, Heuristic_Cost):
		self.Heuristic_Cost = Heuristic_Cost
		self.x = x
		self.Val_Cost = Val_Cost
		self.VelR = 0
		self.y = y
		self.Angle = Angle

		self.VelL = 0
		self.ID_Parent = Parent_ID
		self.id = id

# The heuristic euclidean distance is calculated here in this function
def Distance_Heuristic(x1, y1, x2, y2):

	dist = m.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
	return dist

#This function generates the unique identity fopr each node in order to check for the existing node faster
def ID_Generation(x, y):
	return y*2750 + x*450

#This function will be called whenever the rectangle is needed to drwan in the onstacle space
def draw_rectangle_obst(m1, m2, n1, n2, x, y, Fin_Clearance):


	R1 = [-1, 1, 0, 0]
	R2 = [0, 0, -1, 1]
	R3 = [m1, -m2, n1, -n2]

	draw_rectangle_obst = ((R1[0] * x + R2[0] * y + R3[0] - Fin_Clearance) <= 0) and ((R1[1] * x + R2[1] * y + R3[1] - Fin_Clearance) <= 0) and (
			(R1[2] * x + R2[2] * y + R3[2] - Fin_Clearance) <= 0) and ((R1[3] * x + R2[3] * y + R3[3] - Fin_Clearance) <= 0)

	return draw_rectangle_obst

########The node on the optimal path alone is extracted in this function
def Path_Calculation(Data_SetClosed_Con, Node_Goal):
	#Initializing the nodes list
	List_Node_Final = []
	#X values in the node list is saved in the X_list variable
	X_list = [Node_Goal.x]
	ParentID = Node_Goal.ID_Parent
	# Y values in the node list is saved in the Y_list variable
	Y_list = [Node_Goal.y]

	# Adding all the node points in the list by append function
	List_Node_Final.append(Node_Goal)
	# retracing the path until the parent ID of the node in consideration is -1 which was the ID of source node
	while ParentID != -1:
		node = Data_SetClosed_Con[ParentID]
		List_Node_Final.append(node)
		ParentID = node.ID_Parent
	#Returning the list
	return List_Node_Final

#	This function creates the new node, basically it explores the space by creating new nodes around the current node
def Node_Neighbor_Creation(Points_MapObstacle,Node_Current,  Rad_Wheel,Node_Goal, WheelBase):
	Inc_Time = 0.038  # change in time values

	Angle_O = Node_Current.Angle
	X_goal = Node_Goal.x
	# The assigned values are used later in the algorithm
	Y_goal = Node_Goal.y

	######Storing the Current node start and goal values in the variables
	X_O = Node_Current.x
	Y_O = Node_Current.y

	Val_Cost_old = Node_Current.Val_Cost

	#Defining the RPM values for the algorithm
	RPM1 = 25
	RPM2 = 35

	#Defining the Action space below in a list
	Node_ExpAction = [(0, RPM1), (RPM1, 0), (RPM1, RPM1), (0, RPM2), (RPM2, 0), (RPM2, RPM2), (RPM1, RPM2), (RPM2, RPM1)]
	Node_Neighbors = [] #Initializing the neighbor node list

	for i in range(0, 8): #Exploring for new nodes in all 8 different direction

		WheelVel_Left = (Node_ExpAction[i][0])*(2*m.pi/60)
		WheelVel_Right = (Node_ExpAction[i][1])*(2*m.pi/60)

		#Initializing the parameter with the old x,y,angle values
		x, y, Ang = X_O, Y_O, Angle_O

		# The new values of Angle and x,y position are generated using the equations for
		#differential drive constraints
		for j in range(100):
			#Angle values calculation
			New_Angle = Ang + (WheelVel_Right - WheelVel_Left)*Inc_Time*(Rad_Wheel/WheelBase)
			#New Position value calculation
			Gen_X = (x + (WheelVel_Left + WheelVel_Right)*m.cos(New_Angle)*Inc_Time*(Rad_Wheel/2))
			Gen_Y = (y + (WheelVel_Left + WheelVel_Right)*m.sin(New_Angle)*Inc_Time*(Rad_Wheel/2))
			#Initializing the Pos variable
			Pos = (int(Gen_X), int(Gen_Y))
			#When the poinits are in obstacle we are exiting the loop
			if Points_MapObstacle[Pos]:
				break
			#The new values are stored
			Ang = New_Angle;x = Gen_X;y = Gen_Y
		#At the end of the created node point after 100 steps, the cost values are calculated below
		if j == 99:
			#Converitng to integers
			Gen_X = int(Gen_X)
			Gen_Y = int(Gen_Y)
			#Initializing the Pos variable
			Pos = (Gen_X,Gen_Y)
			# When the poinits are in obstacle we are exiting the loop
			if Points_MapObstacle[Pos]:
				continue
			#The cost value calculated with the old x anfd y points
			Val_Cost_new = Val_Cost_old + Distance_Heuristic(Gen_X, Gen_Y, X_O, Y_O)
			#The heuristic cost calculated with the new point and goal point
			Heuristic_Cost = Val_Cost_new + Distance_Heuristic(Gen_X, Gen_Y, X_goal, Y_goal)

			#When the new X and Y generated exists within the applicable range or workspace
			# and the below mentioned condition is satisfied then the new node is added to the node list
			if (Gen_X < 1110-Fin_Clearance and Gen_X > Fin_Clearance) and (Gen_Y > Fin_Clearance and Gen_Y < 1010-Fin_Clearance):

				ID = ID_Generation(Gen_X, Gen_Y)
				Node_Neighbor = Algorithm_PathCalcNode(Gen_X, Gen_Y, New_Angle, Val_Cost_new, ID, Node_Current.id, Heuristic_Cost)

				#Generating the velocity values for the robot
				# WheelVel_Right = (Gen_X - Node_Current.x)/(Inc_Time*100)
				# WheelVel_Left = (Gen_Y - Node_Current.y) / (Inc_Time * 100)
				Node_Neighbor.VelR = WheelVel_Right
				Node_Neighbor.VelL = WheelVel_Left
				#Appending the node values
				Node_Neighbors.append(Node_Neighbor)

	#print("number of neighbor nodes", len(Node_Neighbors))
	return Node_Neighbors


def Random(X,Y):

	L_V, L_R = 0,0
	X_Old = 0
	PrevY = 0

	C_Vel = []

	for i in range (0,len(X),1):

		x=(X[i])
		y=(Y[i])


		Len = m.sqrt(((x-X_Old)**2)+((y-PrevY)**2))
		Len=int(Len)

		if y>PrevY:
			for i in np.arange (0,2):
				L_V  = 0

				C_Vel.append([L_V, L_R])

			for i in range(0, Len):

				C_Vel.append([L_V, L_R])
			X_Old = x
			PrevY = y


		if y<PrevY:
			for i in range(0, 2):

				L_R = 0
				C_Vel.append([L_V, L_R])

			for i in range(0, Len):

				C_Vel.append([L_V, L_R])
			X_Old = x
			PrevY = y

	return C_Vel

#This function draws the circle whenver called
def drawcircle(m1, n1, Bot_Rad, m2, n2, Fin_Clearance):

	circle_drawn = (m2 - m1) ** 2 + (n2 - n1) ** 2 - ((Fin_Clearance+Bot_Rad) ** 2) <= 0
	return circle_drawn

#The optimal destination threshold for the robot is assigned here
def Dest_Reach(drawcircle_x, drawcircle_y, rad, x, y):
	print ("YesG",drawcircle_x, drawcircle_y)
	print ("Current Pos",x, y)

	if ((x - drawcircle_x) * (x - drawcircle_x) +(y - drawcircle_y) * (y - drawcircle_y) <= rad * rad):
		print ("Reached Next Algorithm_PathCalcNode")
		return True
	else:
		return False

#The Main A-Star algorithm is implemented here
def A_STAR_ALG(start, goal,Points_MapObstacle):

	Data_SetOpen_Con = dict()  # This dictionary holds recent nodes

	# This dictionary holds checked nodes
	Data_SetClosed_Con = dict()
	#Defining a list
	Points_ForPlot = list()

	# Calculating the cost
	Heuristic_Cost_src = Distance_Heuristic(start[0], start[1], goal[0], goal[1])

	# Obtaining the initialized values from the class
	source_node = Algorithm_PathCalcNode(start[0], start[1], 0, 0, 0, -1, Heuristic_Cost_src)
	Node_Goal = Algorithm_PathCalcNode(goal[0], goal[1], 0, 0, 0, 0, 0)

	source_node.VelL = 0
	source_node.VelR = 0
	Node_Goal.VelL = 0
	Node_Goal.VelR = 0

	# Obtaining the ID of the nodes
	source_node.id = ID_Generation(source_node.x, source_node.y)
	Data_SetOpen_Con[source_node.id] = source_node

	# Exploring all the nodes
	while len(Data_SetOpen_Con) != 0:

		# Choosing the Current node ID as the node with the minimium heuristic cost
		id_Node_Current = min(Data_SetOpen_Con, key=lambda i: Data_SetOpen_Con[i].Heuristic_Cost)
		# Obtaining the node values belonging to the ID chosen
		Node_Current = Data_SetOpen_Con[id_Node_Current]

		# Appending the node values
		Points_ForPlot.append((Node_Current.x, Node_Current.y))
		x, y = zip(*Points_ForPlot)

		# Moving the current node id into the set of closed nodes
		del Data_SetOpen_Con[id_Node_Current]
		Data_SetClosed_Con[Node_Current.id] = Node_Current

		# Exiting the function once the goal node or its threhold region is reached
		if (Node_Current.x - Node_Goal.x)**2 + (Node_Current.y - Node_Goal.y)**2 <= 1500:
			Node_Goal.ID_Parent = Node_Current.ID_Parent
			Node_Goal.Val_Cost = Node_Current.Val_Cost
			plt.plot(x, y, "r.")
			break

		# Plotting the generated nodes for every 450th node
		if len(Points_ForPlot) % 450 == 0:
			plt.plot(x, y, "r.")
			plt.pause(0.01)
			Points_ForPlot.clear()

		# Creating the new neighbor nodes
		Node_Neighbors = Node_Neighbor_Creation(Points_MapObstacle,Node_Current, Rad_Wheel,Node_Goal,  WheelBase)

		for neighbor in Node_Neighbors:

			# Going to start of the loop if the neighbor id is already in the closed set of nodes
			if neighbor.id in Data_SetClosed_Con:
				continue

			# On the other hand if the neighbor id is in the open set then checking for the
			# node which has the lowest cost and assigning it as the parent node
			if neighbor.id in Data_SetOpen_Con:
				if Data_SetOpen_Con[neighbor.id].Val_Cost > neighbor.Val_Cost:
					Data_SetOpen_Con[neighbor.id].Val_Cost = neighbor.Val_Cost
					Data_SetOpen_Con[neighbor.id].Heuristic_Cost = neighbor.Heuristic_Cost
					Data_SetOpen_Con[neighbor.id].ID_Parent = id_Node_Current
			# Otherwise assigning this neighbor node into the opendata set
			else:
				Data_SetOpen_Con[neighbor.id] = neighbor

	# Finding the x and y values for the optimal path to be plotted
	List_Node_Final = Path_Calculation(Data_SetClosed_Con, Node_Goal)
	X_list = [d.x for d in List_Node_Final]
	Y_list = [d.y for d in List_Node_Final]
	#Plotting the optimal path
	plt.grid(b=True, which='major', color='y', linestyle='-')
	plt.grid(b=True, which='minor', color='y', linestyle='-')
	plt.plot(X_list, Y_list, c='b')

	#The values are obtained for the V-Rep simulation
	Simulation_Data = [[d.x, d.y, d.Angle, d.VelL, d.VelR] for d in List_Node_Final]



	# Old_X_list = 0
	# Old_Y_list = 0
	# Velocity_X=[]
	# Velocity_Y=[]

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

	# Reversing for the later usage
	Simulation_Data.reverse()
	print(Simulation_Data)
	plt.show()

	vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

	for i in range(0, (len(Simulation_Data))):
		# x = simGetObjectMatrix(Simulation_Data[i+1][2])
		# res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, "remoteApiCommandServer",
		# 																			 vrep.sim_scripttype_childscript,
		# 																			 'RotMatrix', [], [(Simulation_Data[i+1][2])],
		# 																			 [], emptyBuff,
		# 																			 vrep.simx_opmode_blocking)

		errorcode, position = vrep.simxGetObjectPosition(clientID, 189, -1, vrep.simx_opmode_streaming)

		# Obtaining the node X and Y position values
		X_Point = (((Simulation_Data[i][0]) / (100)) - (5.1870))
		Y_Point = (((Simulation_Data[i][1]) / (100)) - (4.8440))
		# X_Point = (((Simulation_Data[i][0]) / (100)))
		# Y_Point = (((Simulation_Data[i][1]) / (100)))

		# Feeding the individual wheel velocities
		VelL = Simulation_Data[i][3]
		VelR = Simulation_Data[i][4]

		# while(round((position[0]),1) is not (round((((Simulation_Data[i + 1][0]) / (100)) - (5.1870)),1)))and (round((position[1]),1) is not (round((((Simulation_Data[i + 1][1]) / (100)) - (4.8440)),1))):
		# while(position[0] is not is_almost_equal(-4.7,-4.9) )and (position[1] is not is_almost_equal(-4.2,-4.5)):
		# while (int((position[0])) is not X_Point) and (int((position[1])) is not Y_Point):
		# j = 0
		# while(1):
		# 	if (isInside(X_Point,Y_Point,0.165,position[0],position[1]) == True):
		# 		vr = 0
		# 		vl = 0
		#
		# 		break

		# print ("PosX,Y",((round((position[0]),1))),(round((position[1]),1)))

		# vr=Simulation_Data[i+1][3]
		# vl=Simulation_Data[i+1][4]

		# print ("Goal Node",(round((((Simulation_Data[i + 1][0]) / (100)) - (5.1870)),1)),(round((Simulation_Data[i + 1][1]) / (100))- (4.8440)),(round((((Simulation_Data[i + 1][1]) / (100)) - (4.8440)),1)))

		# print ("vr,vl)",vr,vl)

		# Feeding the Velocity values to the respective wheels in V-Rep through 'simxSetJointTargetVelocity' function
		errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, VelL, vrep.simx_opmode_streaming)
		errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, VelR, vrep.simx_opmode_streaming)

		# Obtaining the Turtle bot current position through vrep function 'simxGetObjectPosition'
		errorcode, position = vrep.simxGetObjectPosition(clientID, 189, -1, vrep.simx_opmode_streaming)

		# if i ==5:
		# 	exit(0)

		time.sleep(3.8)

	# print("Flag 1")
	# time.sleep(0.013) # loop executes once every 0.2 seconds (= 5 Hz)
	# print ("new_position",position)
	# print('Yes', X_Point, Y_Point)
	# print("Velocity", vr,vl)
	# print(i)

	# Once the goal node and its vicinity is reached we are bringing the robot to stop
	VelL = 0
	VelR = 0
	errorCode = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, VelL, vrep.simx_opmode_streaming)
	errorCode = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, VelR, vrep.simx_opmode_streaming)

	# print("Flag 2")

	return "Execution successful"



if __name__ == "__main__":

	sx, sy, gx, gy = user_input()

	init = time.time()
	if sx < 0 or sy < 0 or gx < 0 or gy < 0 :
		print("\n\nNegative values for node positions and other parameters not allowed")
		exit()


	# plotting the start and goal points
	plt.figure(figsize=(15, 15))
	plt.plot(sx, sy, "rx")
	plt.plot(gx, gy, "rx")

	Points_MapObstacle = Obstacle_mapping((sx,sy),(gx,gy))
	x = []
	y = []
	points = [x for x in Points_MapObstacle.keys() if (Points_MapObstacle[x])]
	x = [i[0] for i in points]
	y = [i[1] for i in points]
	plt.scatter(x, y, s=1, c='k', marker='x')


	# plt.figure(figsize=(15, 15))
	# ax = plt.axes()
	#
	plt.xlabel('x')
	plt.ylabel('y')
	#
	# ax.set_xlim(0, float(1110))
	# ax.set_ylim(0, float(1010))


	plt.title("A-Star Simulation for Differential Drive Robot")



	input = (sx, sy)  # start node points
	final = time.time()

	# Checking whether the input node points are in the obstacle space
	if (sx, sy) not in points and (gx, gy) not in points:
		Success = A_STAR_ALG((sx, sy), (gx, gy), Points_MapObstacle)

	# Asking the user to give some other input points since they are not Valid
	else:
		print(
			" \n*****The start and goal node position values you entered is inside obstacle space or beyond the map boundary*****\n*****Please enter other start and goal node position when you run again*****")
		print(" \n********* Terminating Project Execution now ********* ")
		exit()

	print("Found goal in --> seconds", final - init)
	# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

	print(Success)
	vrep.simxFinish(clientID)
############

