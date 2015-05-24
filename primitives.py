import numpy as np
import matplotlib.pyplot as plt
import collections as co
import cairo
import math
import pdb
import geometry as gm

class Color:
	def __init__(self, r, g, b, a=1.0):
		self.r = r
		self.g = g
		self.b = b
		self.a = a


class CairoData:
	def __init__(self, cr, im):
		self.cr = cr
		self.im = im

	
def get_ball_im(radius=40, fColor=Color(0.0, 0.0, 1.0), sThick=2, sColor=None):
	'''
		fColor: fill color
		sColor: stroke color
		sThick: stroke thickness
	'''
	sz      = 2*(radius + sThick)
	data    = np.zeros((sz, sz, 4), dtype=np.uint8)
	surface = cairo.ImageSurface.create_for_data(data, 
							cairo.FORMAT_ARGB32, sz, sz)
	cr      = cairo.Context(surface)
	#Create a transparent source
	cr.set_source_rgba(1.0, 1.0, 1.0, 0.0)
	cr.paint()
	#Create the border
	cx, cy = radius + sThick, radius + sThick
	cr.arc(cx, cy, radius, 0, 2*math.pi)
	cr.set_line_width(sThick)
	if sColor is not None:
		cr.set_source_rgba(sColor.b, sColor.g, sColor.r, sColor.a) 
	else:
		cr.set_source_rgba(0.0, 0.0, 0.0, 1.0) 
	cr.stroke()
	#Fill in the desired color
	cr.set_source_rgba(fColor.b, fColor.g, fColor.r, fColor.a)
	cr.arc(cx, cy, radius, 0, 2*math.pi)
	cr.fill()
	#cr.destroy()
	return cr, data


def get_rectangle_im(sz=gm.Point(4,100), fColor=Color(1.0, 0.0, 0.0)):
	data    = np.zeros((sz.y(), sz.x(), 4), dtype=np.uint8)
	surface = cairo.ImageSurface.create_for_data(data, 
							cairo.FORMAT_ARGB32, sz.x(), sz.y())
	cr      = cairo.Context(surface)
	#Create a transparent source
	cr.set_source_rgba(1.0, 1.0, 1.0, 0.0)
	cr.paint()
	# Make rectangle and fill in the desired color
	cr.set_source_rgba(fColor.b, fColor.g, fColor.r, fColor.a)
	cr.rectangle(0, 0, sz.x(), sz.y())
	cr.fill()
	#cr.destroy()
	return cr, data


##
#Wall def just defines the physical properties. 
class WallDef:
	def __init__(self,  sz=gm.Point(4, 100), 
							 fColor=Color(1.0, 0.0, 0.0), name=None):
		self.sz        = sz
		self.fColor    = fColor
		self.name      = name
		
##
# Defines the physical properties along with the location etc of the object. 
class Wall:
	def __init__(self, initPos=gm.Point(0, 0), sz=gm.Point(4, 100), 
							 fColor=Color(1.0, 0.0, 0.0), name=None):
		self.sz_     = sz
		self.pos_    = initPos
		self.fColor_ = fColor
		self.name_   = name
		self.make_data()

	@classmethod
	def from_def(cls, wallDef, name, initPos):
		self       = cls(sz=wallDef.sz, fColor=wallDef.fColor)
		self.name_ = name
		self.pos_  = initPos #The upper left corner
		return self

	#Make cairo data
	def make_data(self):
		cr, im = get_rectangle_im(sz=self.sz_, fColor=self.fColor_)
		self.data_      = CairoData(cr, im)	

	#Make the coordinates of the four corners
	def make_coordinates(self):
		self.lTop_ = self.pos_
		self.lBot_ = self.pos_ + gm.Point(0, self.sz_.y())
		self.rTop_ = self.pos_ + gm.Point(self.sz_.x(), 0)
		self.rBot_ = self.pos_ + gm.Point(self.sz_.x(), self.sz_.y())
		self.l1_   = Line(self.lTop_, self.lBot_) #Left line
		self.l2_   = Line(self.lBot_, self.rBot_)
		self.l3_   = Line(self.rBot_, self.rTop_)
		self.l4_   = Line(self.rTop_, self.lTop_)

	#Imprint the wall
	def imprint(self, cr, xSz, ySz):
		'''
			xSz, ySz: Size of the canvas on which imprint has to be made. 
		'''
		y, x  = self.pos_.y(), self.pos_.x()		
		srcIm = np.zeros((ySz, xSz, 4), dtype=np.uint8)
		srcIm[y : y + self.sz_.y(), x : x + self.sz_.x(),:] = self.data_.im[:] 
		surface = cairo.ImageSurface.create_for_data(srcIm, 
								cairo.FORMAT_ARGB32, xSz, ySz)
		cr.set_source_surface(surface)		
		cr.rectangle(x, y, self.sz_.x(), self.sz_.y())
		cr.fill()
		print "Wall- x: %d, y: %d, szX: %d, szY: %d" % (x, y, self.sz_.x(), self.sz_.y())

	#Gives the normal that are needed to solve for collision. 
	def get_collision_normal(self, pt):
		'''
			The wall has 4 faces and a normal associated with each of these faces.
			we would basically determine where the point lies and then generate
			collisions based on that. 
		'''	
		#If the particle is to the left. 
		if ((self.l1_.get_point_location(pt) == -1 and self.l3_.get_point_location(pt)==1) or
				 self.l1_.get_point_location(pt) == 0):
			return gm.Point((-1,0))
		
		#If the particle is on the right. 	
		if ((self.l1_.get_point_location(pt) == 1 and self.l3_.get_point_location(pt) == -1) or
				 self.l3_.get_point_location(pt) == 0):
			return gm.Point((1,0))

		#If the particle is on the top
		if ((self.l2_.get_point_location(pt) == 1 and self.l4_.get_point_location(pt) == -1) or
				 self.l4_.get_point_location(pt) == 0):
			return gm.Point((0,1))

		#If the particle is on the bottom
		if ((self.l2_.get_point_location(pt) == -1 and self.l4_.get_point_location(pt) == 1) or:
				 self.l2_.get_point_location(pt) == 0):
			return gm.Point((0,-1))

	def name(self):
		return self.name_


##
#Ball def just defines the physical properties. 
class BallDef:
	def __init__(self, radius=20, sThick=2, 
							 sColor=Color(0.0, 0.0, 0.0), fColor=Color(1.0, 0.0, 0.0),
							 name=None):
		self.radius    = radius
		self.sThick    = sThick
		self.sColor    = sColor
		self.fColor    = fColor
		self.name      = name

		
##
# Defines the physical properties along with the location etc of the object. 
class Ball:
	def __init__(self, radius=20, sThick=2, 
							 sColor=Color(0.0, 0.0, 0.0), fColor=Color(1.0, 0.0, 0.0),
							 name=None, initPos=gm.Point(0,0), initVel=gm.Point(0,0)):
		self.radius_    = radius
		self.sThick_    = sThick
		self.sColor_    = sColor
		self.fColor_    = fColor
		self.name_      = name
		self.pos_       = initPos
		self.tCol_      = 0
		self.vel_       = initVel
		self.make_data()

	@classmethod
	def from_def(cls, ballDef, name, initPos, initVel=gm.Point(0,0)):
		self = cls(radius=ballDef.radius, sThick=ballDef.sThick, 
							 sColor=ballDef.sColor, fColor=ballDef.fColor)
		self.name_ = name
		self.pos_  = initPos
		self.vel_  = initVel
		return self

	def set_name(self, name):
		self.name_ = name

	#Make cairo data
	def make_data(self):
		cr, im      = get_ball_im(radius=self.radius_, fColor=self.fColor_,
														  sThick=self.sThick_, sColor=self.sColor_)
		self.data_  = CairoData(cr, im)	
		self.ySz_, self.xSz_   = im.shape[0], im.shape[1]
		self.yOff_, self.xOff_ = np.floor(self.ySz_ / 2), np.floor(self.xSz_ / 2)

	#Imprint the ball
	def imprint(self, cr, xSz, ySz):
		#Get the position of bottom left corner.
		y, x  = self.pos_.y() - self.yOff_, self.pos_.x() - self.xOff_		
		srcIm = np.zeros((ySz, xSz, 4), dtype=np.uint8)
		srcIm[y:y+self.ySz_, x:x+self.xSz_,:] = self.data_.im[:] 
		surface = cairo.ImageSurface.create_for_data(srcIm, 
								cairo.FORMAT_ARGB32, xSz,ySz)
		cr.set_source_surface(surface)		
		cr.rectangle(x, y, self.xSz_, self.ySz_)
		cr.fill()

	def name(self):
		return self.name_

	def get_position(self):
		return self.pos_
	
	def get_velocity(self):
		return self.vel_

	def set_position(self, pos):
		self.pos_ = pos

	def set_velocity(self, vel):
		self.vel_ = vel


class Dynamics():
	def __init__(self, world, g=0, deltaT=0.01):
		'''
			g: gravity, since the (0,0) is top left, gravity will be positive. 
		'''
		self.world_  = world
		self.g_      = gm.Point(0, g)
		self.deltaT_ = deltaT
		#Record which objects have been stepped and which have not been.
		self.isStep_ = co.OrderedDict()
		#Record time to collide and object of collision
		self.tCol_   = co.OrderedDict()
		self.objCol_ = co.OrderedDict()
		for name in self.get_dynamic_object_names():
			self.isStep_[name] = False
			self.tCol_[name] = 0
			self.objCol_[name] = None

	#Set gravity
	def set_g(self, g):
		self.g_ = g

	#Get names of objects
	def get_dynamic_object_names(self):
		return self.world_.get_dynamic_object_names()

	def step_object(self, obj, name):
		if self.isStep_[name]:
			return
		if self.tCol_[name] < self.deltaT_:
			self.resolve_collision(obj, name)
			return
		pos = obj.get_position()
		vel = obj.get_velocity()
		velMag = vel.mag()
		#Update position: s = ut + 0.5at^2
		pos = pos + self.v_.scale(self.deltaT_) + self.g_.scale(0.5 * self.deltaT_ * self.deltaT_)	
		#Update velocity: v = u + at
		vel = vel + self.g_.scale(self.deltaT_)
		#If a sationary object has been set to motion, then perform resolve_collision
		if velMag == 0 and vel.mag() > 0:
			self.resolve_collision(obj, name) 
		self.isStep_[name] = True

	#Step the entire world
	def step(self):
		#Reset step states
		for name in self.get_dynamic_object_names():
			self.isStep_[name] = False
		#Step every object
		for name in self.get_dynamic_object_names():
			self.step_object(self.world_.get_object(name), name)
		 
	#Resolve the collisions
	def resolve_collision(self, obj, name):
		pos = obj.get_position()
		vel = obj.get_velocity()
		#Stationary object
		if vel.mag() == 0:
			self.tCol_[name] = np.inf
			self.step_object(self, obj, name)
			return	
		#Stationary object just set into motion
		if self.tCol_[name]==np.inf and vel.mag() > 0:
			#Detect Collisions

	def detect_collision(self, los, obj):
	'''
		los: Line of Sight - the direction of velocity vector of object 1
		obj: The second object.
		     if corners of bbox of obj lie on opposite sides of los then there
				 will be a collision. 
	'''
			

	#Get time to collision
	def get_time_to_collide(self, obj, name):
		#The line that represents the motion vector of the object
		vel = obj.get_velocity()
		los = Line(gm.Point(0,0), vel)			
		
		allNames = self.world_.get_object_names()
		for an in allNames:
							

#The world
class World:
	def __init__(self, xSz=200, ySz=200, deltaT=0.1):
		'''
			xSz, ySz: Size of the cavas that defines the world.
			deltaT: the stepping time used in the simulation. 
		'''
		#Static Objects
		self.static_  = co.OrderedDict()
		#Dynamic Objects
		self.dynamic_ = co.OrderedDict()
		#gm.Pointers to all objects
		self.objects_ = co.OrderedDict()
		#Count of objects
		self.count_   = co.OrderedDict()
		self.init_count()
		#Form the base canvas
		self.xSz_     = xSz
		self.ySz_     = ySz 
		cr, im        = get_rectangle_im(sz=gm.Point(xSz,ySz), fColor=Color(1.0,1.0,1.0))
		self.baseCanvas_ = CairoData(cr, im)

	#Contains the names of primitives that the world
	# can have.
	def init_count(self):
		self.count_['wall'] = 0
		self.count_['ball'] = 0

	#Names of dynamic objects
	def get_dynamic_object_names(self):
		return self.dynamic_.keys()

	#Get object names
	def get_object_names(self):
		return self.objects_.keys()

	##
	def get_object(self, name):
		assert name in self.objects_.keys(), 'Object: %s not found' % name
		return self.objects_[name]

	##
	def get_object_name_type(self, objDef, initPos):
		if isinstance(objDef, WallDef):
			name    = 'wall-%d' % self.count_['wall']
			obj     = Wall.from_def(objDef, name, initPos)
			objType = 'static' 
			self.count_['wall'] += 1
		elif isinstance(objDef, BallDef):
			name    = 'ball-%d' % self.count_['ball']					
			obj     = Ball.from_def(objDef, name, initPos)
			objType = 'dynamic'
			self.count_['ball'] += 1
		else:
			raise Exception('Unrecognized object type')
		
		return obj, name, objType

	##
	def add_object(self, objDef, initPos=gm.Point(0,0)):
		'''
			objDef : The definition of the object that needs to be added.
			initPos: The initial position of the object in the normalized coords.
		'''
		obj, name, objType = self.get_object_name_type(objDef, initPos)
		if name in self.objects_.keys():
			raise Exception('Object with name %s already exists' % name)
		self.objects_[name] = obj
		if objType == 'static':
			self.static_[name]  = obj
		else:
			self.dynamic_[name] = obj		
	
	def set_object_position(self, objName, newPos):
		assert objName in self.dynamic_.keys()
		self.dynamic_[objName].set_position(newPos)

	def increment_object_position(self, objName, deltaPos):
		assert objName in self.dynamic_.keys()
		self.dynamic_[objName].set_position(self.dynamic_[objName].get_position() + deltaPos)

	def generate_image(self):
		data    = np.zeros((self.ySz_, self.xSz_, 4), dtype=np.uint8)
		data[:] = self.baseCanvas_.im[:]
		surface = cairo.ImageSurface.create_for_data(data, 
								cairo.FORMAT_ARGB32, self.xSz_, self.ySz_)
		cr      = cairo.Context(surface)
		for key, obj in self.objects_.iteritems():
			obj.imprint(cr, self.xSz_, self.ySz_)
		return data
