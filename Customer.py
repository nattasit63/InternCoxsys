class Customer:
	def __init__(self,i,x,y,q):
		self.i = i
		self.x = x
		self.y = y
		self.q = q # demand for this customer
	def clearr(self):
		self.i = 0
		self.x = 0
		self.y = 0
		self.q = 0
	def __repr__(self):
		return str(self.i)