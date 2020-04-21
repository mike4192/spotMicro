import math
from math import pi, sin, cos
import matplotlib.pyplot as plt
import numpy as np
from .utilities import spot_micro_kinematics as smk
from .utilities import transformations

d2r = pi/180
r2d = 180/pi

class SpotMicroLeg(object):
    '''Encapsulates a spot micro leg that consists of 3 links and 3 joint angles
    
    Attributes:
        _q1: Rotation angle in radians of hip joint
        _q2: Rotation angle in radians of upper leg joint
        _q3: Rotation angle in radians of lower leg joint
        _l1: Length of leg link 1 (i.e.: hip joint)
        _l2: Length of leg link 2 (i.e.: upper leg)
        _l3: Length of leg link 3 (i.e.: lower leg)
        _ht_leg: Homogeneous transformation matrix of leg starting 
                 position and coordinate system relative to robot body.
                 4x4 np matrix  
        _leg12: Boolean specifying whether leg is 1 or 2 (rightback or rightfront)
                or 3 or 4 (leftfront or leftback)  
    '''

    def __init__(self,q1,q2,q3,l1,l2,l3,ht_leg_start,leg12):
        '''Constructor'''
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3
        self._l1 = l1
        self._l2 = l2
        self._l3 = l3
        self._ht_leg_start = ht_leg_start
        self._leg12 = leg12

        # Create homogeneous transformation matrices for each joint
        self._t01 = smk.t_0_to_1(self._q1,self._l1)
        self._t12 = smk.t_1_to_2()
        self._t23 = smk.t_2_to_3(self._q2,self._l2)
        self._t34 = smk.t_3_to_4(self._q3,self._l3)


    def set_angles(self,q1,q2,q3):
        '''Set the three leg angles and update transformation matrices as needed'''
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3
        self._t01 = smk.t_0_to_1(self._q1,self._l1)
        self._t23 = smk.t_2_to_3(self._q2,self._l2)
        self._t34 = smk.t_3_to_4(self._q3,self._l3)
    
    def set_homog_transf(self,ht_leg_start):
        '''Set the homogeneous transformation of the leg start position'''
        self._ht_leg_start = ht_leg_start

    def get_homog_transf(self):
        '''Return this leg's homogeneous transformation of the leg start position'''
        return self._ht_leg_start

    def set_foot_position_in_local_coords(self,x4,y4,z4):
        '''Set the position of the foot by computing joint angles via inverse kinematics from inputted coordinates.
        Leg's coordinate frame is the frame defined by self._ht_leg_start

        Args:
            x4: Desired foot x position in leg's coordinate frame
            y4: Desired foot y position in leg's coordinate frame
            z4: Desired foot z position in leg's coordinate frame
        Returns:
            Nothing
        '''
        # Run inverse kinematics and get joint angles
        leg_angs = smk.ikine(x4,y4,z4,self._l1,self._l2,self._l3,self._leg12)

        # Call method to set joint angles for leg
        self.set_angles(leg_angs[0],leg_angs[1],leg_angs[2])

    def set_foot_position_in_global_coords(self,x4,y4,z4):
        ''' Set the position of the foot by computing joint angles via inverse kinematics from inputted coordinates.
        Inputted coordinates in the global coordinate frame

        Args:
            x4: Desired foot x position in global coordinate frame
            y4: Desired foot y position in global coordinate frame
            z4: Desired foot z position in global coordinate frame
        Returns:
            Nothing
        '''
        # Get inverse of leg's homogeneous transform
        ht_leg_inv = transformations.ht_inverse(self._ht_leg_start)

        # Convert the foot coordinates for use with homogeneous transforms, e.g.:
        # p4 = [x4, y4, z4, 1]
        p4_global_coord = np.block( [np.array([x4, y4, z4]), np.array([1])])

        # Calculate foot coordinates in each leg's coordinate system
        p4_in_leg_coords = ht_leg_inv.dot(p4_global_coord) 

        # Call this leg's position set function for coordinates in local frame
        self.set_foot_position_in_local_coords(p4_in_leg_coords[0],p4_in_leg_coords[1],p4_in_leg_coords[2])


    def get_leg_points(self):
        '''Get coordinates of 4 points that define a wireframe of the leg:
            Point 1: hip/body point
            Point 2: upper leg/hip joint
            Point 3: Knee, (upper/lower leg joint)
            Point 4: Foot, leg end
        
        Returns:
            A length 4 tuple consisting of 4 length 3 numpy arrays representing the 
            x,y,z coordinates in the global frame of the 4 leg points
        '''
        # Build up the total homogeneous transformation incrementally, saving each leg
        # point along the way
        # The total homogeneous transformation builup is:
        # ht = ht_leg_start @ t01 @ t12 @ t23 @ t34 
        p1 = self._ht_leg_start[0:3,3]

        # ht_buildup = self._ht_leg_start @ self._t01 @ self._t12
        ht_buildup = np.matmul(np.matmul(self._ht_leg_start, self._t01), self._t12)

        p2 = ht_buildup[0:3,3]

        # ht_buildup = ht_buildup @ self._t23
        ht_buildup = np.matmul(ht_buildup, self._t23)
        
        p3 = ht_buildup[0:3,3]

        # ht_buildup = ht_buildup @ self._t34
        ht_buildup = np.matmul(ht_buildup, self._t34)

        p4 = ht_buildup[0:3,3]

        return (p1,p2,p3,p4)

    def get_foot_position_in_global_coords(self):
        ''' Return coordinates of the foot in the leg's local coordinate frame'''
        # ht_foot = self._ht_leg_start @ self._t01 @ self._t12 @ self._t23 @ self._t34
        ht_foot = np.matmul(np.matmul(np.matmul(np.matmul(self._ht_leg_start, self._t01), self._t12), self._t23), self._t34)
        return ht_foot[0:3,3]
    
    def get_leg_angles(self):
        '''Return leg angles as a tuple of 3 angles, (q1, q2, q3)'''

        return (self._q1,self._q2,self._q3)


class SpotMicroStickFigure(object):
    """Encapsulates an 12 DOF spot micro stick figure  

    Encapuslates a 12 DOF spot micro stick figure. The 12 degrees of freedom represent the 
    twelve joint angles. Contains inverse kinematic capabilities
    
    Attributes:
        hip_length: Length of the hip joint
        upper_leg_length: length of the upper leg link
        lower_leg_length: length of the lower leg length
        body_width: width of the robot body
        body_height: length of the robot body

        x: x position of body center
        y: y position of body center
        z: z position of body center

        phi: roll angle in radians of body
        theta: pitch angle in radians of body
        psi: yaw angle in radians of body

        ht_body: homogeneous transformation matrix of the body

        rightback_leg_angles: length 3 list of joint angles. Order: hip, leg, knee
        rightfront_leg_angles: length 3 list of joint angles. Order: hip, leg, knee
        leftfront_leg_angles: length 3 list of joint angles. Order: hip, leg, knee
        leftback_leg_angles: length 3 list of joint angles. Order: hip, leg, knee

        leg_rightback
        leg_rightfront
        leg_leftfront
        leg_leftback
        
    """
    def __init__(self,x=0,y=.18,z=0,phi=0,theta=0,psi=0):
        '''constructor'''
        self.hip_length = 0.055
        self.upper_leg_length = 0.1075
        self.lower_leg_length = 0.130
        self.body_width = 0.078
        self.body_length = 0.186

        self.x = x
        self.y = y
        self.z = z
        
        self.phi = phi
        self.theta = theta
        self.psi = psi   

        # Initialize Body Pose
        # Convention for this class is to initialize the body pose at a x,y,z position, with a phi,theta,psi orientation
        # To achieve this pose, need to apply a homogeneous translation first, then a homgeneous rotation
        # If done the other way around, a coordinate system will be rotate first, then translated along the rotated coordinate system
        # self.ht_body = transformations.homog_transxyz(self.x,self.y,self.z) @ transformations.homog_rotxyz(self.phi,self.psi,self.theta)
        self.ht_body = np.matmul(transformations.homog_transxyz(self.x,self.y,self.z), transformations.homog_rotxyz(self.phi,self.psi,self.theta))
        
        # Intialize all leg angles to 0, 30, 30 degrees
        self.rb_leg_angles   = [0,-30*d2r,60*d2r]
        self.rf_leg_angles   = [0,-30*d2r,60*d2r]
        self.lf_leg_angles   = [0,30*d2r,-60*d2r]
        self.lb_leg_angles   = [0,30*d2r,-60*d2r]

        # Create a dictionary to hold the legs of this spot micro object.
        # First initialize to empty dict
        self.legs = {}

        self.legs['leg_rightback'] =     SpotMicroLeg(self.rb_leg_angles[0],self.rb_leg_angles[1],self.rb_leg_angles[2],
                                                     self.hip_length,self.upper_leg_length,self.lower_leg_length,
                                                     smk.t_rightback(self.ht_body,self.body_length,self.body_width),leg12=True) 
        
        self.legs['leg_rightfront'] =   SpotMicroLeg(self.rf_leg_angles[0],self.rf_leg_angles[1],self.rf_leg_angles[2],
                                                     self.hip_length,self.upper_leg_length,self.lower_leg_length,
                                                     smk.t_rightfront(self.ht_body,self.body_length,self.body_width),leg12=True)
                                                  
        self.legs['leg_leftfront'] =    SpotMicroLeg(self.lf_leg_angles[0],self.lf_leg_angles[1],self.lf_leg_angles[2],
                                                     self.hip_length,self.upper_leg_length,self.lower_leg_length,
                                                     smk.t_leftfront(self.ht_body,self.body_length,self.body_width),leg12=False)

        self.legs['leg_leftback'] =     SpotMicroLeg(self.lb_leg_angles[0],self.lb_leg_angles[1],self.lb_leg_angles[2],
                                                     self.hip_length,self.upper_leg_length,self.lower_leg_length,
                                                     smk.t_leftback(self.ht_body,self.body_length,self.body_width),leg12=False) 

    def get_leg_coordinates(self):
        '''Return coordinates of each leg as a tuple of 4 sets of 4 leg points'''
        
        return (self.legs['leg_rightback'].get_leg_points(),
                self.legs['leg_rightfront'].get_leg_points(),
                self.legs['leg_leftfront'].get_leg_points(),
                self.legs['leg_leftback'].get_leg_points())

    def set_leg_angles(self,leg_angs):
        ''' Set the leg angles for all four legs

        Args:
            leg_angs: Tuple of 4 lists of leg angles. Legs in the order rightback
                      rightfront, leftfront, leftback. ANgles in the order q1,q2,q3.
                      An example input:
                        ((rb_q1,rb_q2,rb_q3),
                         (rf_q1,rf_q2,rf_q3),
                         (lf_q1,lf_q2,lf_q3),
                         (lb_q1,lb_q2,lb_q3))

        Returns:
            Nothing
        '''
        self.legs['leg_rightback'].set_angles(leg_angs[0][0],leg_angs[0][1],leg_angs[0][2])
        self.legs['leg_rightfront'].set_angles(leg_angs[1][0],leg_angs[1][1],leg_angs[1][2])
        self.legs['leg_leftfront'].set_angles(leg_angs[2][0],leg_angs[2][1],leg_angs[2][2])
        self.legs['leg_leftback'].set_angles(leg_angs[3][0],leg_angs[3][1],leg_angs[3][2])            


    def set_absolute_foot_coordinates(self,foot_coords):
        '''Set foot coordinates to a set inputted in the global coordinate frame and compute 
        and set the joint angles to achieve them using inverse kinematics
        
        Args:
            foot_coords: A 4x3 numpy matrix of desired (x4,y4,z4) positions for the end point (point 4) of each of
                    the four legs. I.e., the foot.
                    Leg order: rigthback, rightfront, leftfront, leftback. Example input:
                        np.array( [ [x4_rb,y4_rb,z4_rb],
                                    [x4_rf,y4_rf,z4_rf],
                                    [x4_lf,y4_lf,z4_lf],
                                    [x4_lb,y4_lb,z4_lb] ])
        Returns:
            Nothing
        '''

        # For each leg, call its method to set foot position in global coordinate frame
        
        foot_coords_dict = {'leg_rightback':foot_coords[0],
                            'leg_rightfront':foot_coords[1],
                            'leg_leftfront':foot_coords[2],
                            'leg_leftback':foot_coords[3]}
        
        for leg_name in self.legs:
            x4 = foot_coords_dict[leg_name][0]
            y4 = foot_coords_dict[leg_name][1]
            z4 = foot_coords_dict[leg_name][2]
            self.legs[leg_name].set_foot_position_in_global_coords(x4,y4,z4)

    def set_absolute_body_pose(self, ht_body):
        '''Set absolute pose of body, while holding foot positions in place'''
        
        # Get current foot position of each leg in global coordinate system
        # These are 1x3 numpy arrays
        foot_coords = {}
        for leg_name in self.legs:
            foot_coords[leg_name] = self.legs[leg_name].get_foot_position_in_global_coords()

        # Set body pose
        self.ht_body = ht_body

        # Update each leg's homogeneous transformation 
        self.legs['leg_rightback'].set_homog_transf(smk.t_rightback(self.ht_body,self.body_length,self.body_width))
        self.legs['leg_rightfront'].set_homog_transf(smk.t_rightfront(self.ht_body,self.body_length,self.body_width))
        self.legs['leg_leftfront'].set_homog_transf(smk.t_leftfront(self.ht_body,self.body_length,self.body_width))
        self.legs['leg_leftback'].set_homog_transf(smk.t_leftback(self.ht_body,self.body_length,self.body_width))

        # Prep foot coordinates to call method to set absolute foot coordinates
        foot_coords_matrix = np.block([ [foot_coords['leg_rightback']],
                                        [foot_coords['leg_rightfront']],
                                        [foot_coords['leg_leftfront']],
                                        [foot_coords['leg_leftback']]  ])

        self.set_absolute_foot_coordinates(foot_coords_matrix)

    def set_body_angles(self,phi=0,theta=0,psi=0):
        '''Set a body angles without translation of the body

        Args:
            phi: roll angle in radians
            theta: pitch angle in radians
            psi: yaw angle in radians
        Returns:
            Nothing
        '''
        # Create a xyz rotation matrix
        r_xyz = transformations.rotxyz(phi,psi,theta)

        # Get current body pose, and replace rotation part with r_xyz
        ht_body = self.ht_body

        ht_body[0:3,0:3] = r_xyz

        # Call method to set absolute body pose
        self.set_absolute_body_pose(ht_body)

    def get_leg_angles(self):
        ''' Get the leg angles for all four legs
        Args:
            None
        Returns:
            leg_angs: Tuple of 4 of the leg angles. Legs in the order rightback
                      rightfront, leftfront, leftback. Angles in the order q1,q2,q3.
                      An example output:
                        ((rb_q1,rb_q2,rb_q3),
                         (rf_q1,rf_q2,rf_q3),
                         (lf_q1,lf_q2,lf_q3),
                         (lb_q1,lb_q2,lb_q3))
        '''
        return (    self.legs['leg_rightback'].get_leg_angles(),
                    self.legs['leg_rightfront'].get_leg_angles(),
                    self.legs['leg_leftfront'].get_leg_angles(),
                    self.legs['leg_leftback'].get_leg_angles()     )


    def print_leg_angles(self):
        ''' Print the joint angles for alll four legs'''
        return None