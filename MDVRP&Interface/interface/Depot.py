
class Depot:
	
	def __init__(self,i,x,y,D,Q,m,t):
		self.prev = []
		self.i = i
		self.x = x
		self.y = y
		if D == 0:
			self.D = float('inf')
		else:
			self.D = D # maximum duration of a route
		self.Q = Q # allowed maximum load of a vehicle (same for all the vehicles assigned to all depots) 
		self.m = m # maximum number of vehicles available in each depot
		self.customer_list = []
		self.true_customer_list = []
		self.total_depot = t
		self.vehicles = [[]]
	
	def clearr(self):
		self.i = 0
		self.x = 0
		self.y = 0	
		self.D = 0
		self.m = 0 
		self.customer_list = []
		self.vehicles = [[]]
		self.depot_id=[]

	def __repr__(self):
		return str(self.i)

	# Route scheduling
	def route_schedule(self):
		
		vehicle = 0				
		for c in self.customer_list:
				self.vehicles[vehicle].append(c)
				if self.route_duration(vehicle) > self.D or self.vehicle_load(vehicle) > self.Q:
					self.vehicles[vehicle].remove(c)
					vehicle += 1
					self.vehicles.append([c])
	
		

	def update_customer_list(self):
		self.customer_list = []
		for vehicle in range(len(self.vehicles)):
			for c in self.vehicles[vehicle]:
				self.customer_list.append(c)

	def update_routes(self):
		counter = 0
		for vehicle in range(len(self.vehicles)):
			inner_counter = 0
			for c in self.vehicles[vehicle]:
				self.vehicles[vehicle][inner_counter] = self.customer_list[counter]
				inner_counter += 1
				counter += 1
	# Working 
	def route_duration(self,vehicle):
		duration = 0
		if len(self.vehicles[vehicle]) > 1:
			duration += self.distance(self,self.vehicles[vehicle][0])
			for i in range(1,len(self.vehicles[vehicle])):
				duration += self.distance(self.vehicles[vehicle][i-1],self.vehicles[vehicle][i])
			duration += self.distance(self.vehicles[vehicle][-1],self)
		elif len(self.vehicles[vehicle]) == 1:
			duration = self.distance(self,self.vehicles[vehicle][0])*2
		return duration

	# Vehicle load
	def vehicle_load(self,vehicle):
		return sum(c.q for c in self.vehicles[vehicle])

	# Depot load
	def get_load(self):
		return sum(c.q for c in self.customer_list)

	# Distance 
	def distance(self,c1,c2):
		# print(c1 ,c2)
		shift = self.total_depot-1
		distance_matrix = [
			[0, 5, 25, 7, 68, 34, 87, 91, 3, 10], 
			[57, 0, 30, 30, 30, 50, 22, 65, 6, 25], 
			[97, 53, 0, 50, 26, 69, 9999, 9999, 9999, 9999], #1
			[36, 64, 16, 0, 71, 73, 9999, 9999, 9999, 9999],   #2
			[84, 73, 97, 77, 0, 75, 9999, 9999, 9999, 9999], 	#3
			[44, 93, 70, 7, 50, 0,9999, 9999, 9999, 9999], 	#4
			[72, 15,9999, 9999, 9999, 9999, 0, 8, 74, 41], 	#5
			[56, 62, 9999, 9999, 9999, 9999, 24, 0, 20, 97], 	#6
			[52, 72,9999, 9999, 9999, 9999 ,5, 26, 0, 70], 		#7
			[42, 97, 9999, 9999, 9999, 9999, 86, 59, 71, 0]]	#8
		
		pre = int(str(c1))
		after =  int(str(c2))
		cost = distance_matrix[pre+shift][after+shift]
		# print((pre,after),'   =  ',cost)
		
		

		# return cost
		# print(((c1.x - c2.x)**2 + (c1.y - c2.y)**2)**0.5)
		return ((c1.x - c2.x)**2 + (c1.y - c2.y)**2)**0.5
		