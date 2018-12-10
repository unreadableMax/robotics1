-- ||-----------------------------------------------------------------||
--                      ...::: Exercice 3 :::...
-- ||-----------------------------------------------------------------||

-- ::External dependencies:: 
math = require('math')

-- ||-----------------------------------------------------------------||
-- || System Parameters |

-- ::rod lengths::
local l_1 = 0.40  -- [m]  <--- add value here 0.50
local l_2 = 0.60  -- [m]  <--- add value here  0.50

-- ::sphere radian::
local r_1 = 0.08  -- [m]  <--- add value here  0.08
local r_2 = 0.10  -- [m]  <--- add value here  0.08

-- ::density::
local rho_1 = 7774 -- [kg/m³]  <--- add value here  1000
local rho_2 = 7774 -- [kg/m³]  <--- add value here  1000

-- ||-----------------------------------------------------------------||

-- ---------------------------------------------------------------------
--   FUNCTION:: rotationalInertia_sphere -> Task a)
-- ---------------------------------------------------------------------
function rotationalInertia_sphere(_r, _rho, _l)  -- what does this underscore-stuff mean? is it some kind of syntax?
   --local theta_s = math.pow(_r,5)
   local m = mass_sphere(_r, _rho)
   local theta_s = (2/5)*m*_r*_r + m*_l*_l
   return theta_s
end

-- ---------------------------------------------------------------------

-- ---------------------------------------------------------------------
--   FUNCTION:: mass_sphere -> Task a)
-- ---------------------------------------------------------------------
function mass_sphere(_r, _rho) -- original version
   --local m_s = math.pow(_r,3)
   local pi  = 3.141592
   local V   = (4/3)*pi*_r*_r
   local m_s = _rho*V
   return m_s
end

-- ---------------------------------------------------------------------

-- ||-----------------------------------------------------------------||
-- || Meshes |

meshes = {
   -- ---------------------------------------
   -- :: Base - Meshes ::
   BaseBlock = {
      name = "BaseBlock",
      src = "meshes/unit_cube.obj",
      color = { 0.4, 0.4, 0.4},
      dimensions = { 0.2, 0.2, 0.04 },
      mesh_center = { -0.15, 0, 0.0 },
   },
   BaseHinge = {
      name = "BaseHinge",
      color = { 0.4, 0.4, 0.4},
      src = "meshes/unit_cube.obj",
      dimensions = { 0.1, 0.04, 0.04 },
      mesh_center = { -0.05, 0, 0.0 },
   },
   Joint = {
      name = "Joint",
      color = { 0.8, 0.8, 0.8},
      src = "meshes/unit_sphere_medres.obj",
      dimensions = { 0.08, 0.08, 0.08 },
      mesh_center = { 0.0, 0, 0.0 },
   },
   -- ---------------------------------------
   -- :: Segment A - Meshes ::
   segmentA = {
      name = "segmentA",
      color = { 0.5, 0.5, 0.5},
      src = "meshes/unit_cube.obj",
      dimensions = { 0.05, 0.05, l_1 }, 
      mesh_center = { 0.0,0, -l_1/2.0 }, 
   },
   massA = {
      name = "MassA",
      color = { 0.0, 0.0, 1.0}, 
      src = "meshes/unit_sphere_medres.obj",
      dimensions = { r_1*2.0, r_1*2.0, r_1*2.0 },
      mesh_center = { 0, 0.0, -l_1 },            
   },
   -- ---------------------------------------
   -- :: Segment B - Meshes ::
   segmentB = {
      name = "SegmentB",
      color = { 0.5, 0.5, 0.5},
      src = "meshes/unit_cube.obj",
      dimensions = { 0.05, 0.05, l_2 },     
      mesh_center = { 0.0, 0.0, -l_2/2.0 }, 
   },
   massB = {
      name = "MassB",
      color = { 0.0, 0.0, 1.0},
      src = "meshes/unit_sphere_medres.obj",
      dimensions = { r_2*2.0, r_2*2.0, r_2*2.0 }, 
      mesh_center = { 0.0, 0.0, -l_2},          
   },
}

-- ||-----------------------------------------------------------------||
-- || Dynamics |

local I_s1 = rotationalInertia_sphere(r_1,rho_1,l_1)
local I_s2 = rotationalInertia_sphere(r_2,rho_2,l_2)

dynamics = {
   -- ---------------------------------------
   -- :: Segment A - Dynamics::
   segmentA = {
      mass =  mass_sphere(r_1, rho_1),
      I_s = rotationalInertia_sphere(r_1,rho_1,l_1),

      com = { 0.0, 0.0, -l_1}, 
      inertia = {{I_s1, 0, 0},   
				{0, I_s1, 0},
				{0, 0, I_s1}}
   },
   -- ---------------------------------------
   -- :: Segment B - Dynamics::
   segmentB = {
      mass =  mass_sphere(r_2, rho_2),
      com = { 0.0, 0.0, -l_2}, 
      inertia = {{I_s2, 0, 0}, 
				{0, I_s2, 0},
				{0, 0, I_s2}}
   }
}

-- ||-----------------------------------------------------------------||
-- || Model |

model = {
   -- ---------------------------------------
   -- :: Configuration for visualization ::
   configuration = {
      axis_front = { 1, 0, 0 },
      axis_up    = { 0, 0, 1 },
      axis_right = { 0, -1, 0 },
      rotation_order = { 2, 1, 0},
   },

   -- ---------------------------------------
   -- :: Standard dynamic configuration ::
   gravity = { 0., 0., -9.81}, 

   -- ---------------------------------------
   -- :: Multi-limb structure ::
   frames = {
      -- ------------------------------------
      -- :: Frame::Base
      --
      -- NOTE:: leave unchanged !!
      {
	 -- :: tree identification
	 name = "Base",
	 parent = "ROOT",
	 -- :: kinematics
	 joint_frame = {
	    r = { 0, 0 , (l_1+l_2)*1.5 },
	 },
	 -- :: visualization
	 visuals = {
	    meshes.BaseBlock,
	    meshes.BaseHinge,
	 },
      },
      -- ------------------------------------
      -- :: Frame::Segment A
      {
	 -- :: tree identification
	 name = "SegmentA",
	 parent = "Base",
	 -- :: dynamics
	 body = dynamics.segmentA,
	 -- :: kinematics
	 joint_frame = {
	    r = { 0, 0 , 0.0 },    
	 },
	 joint = { 
	    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0} -- 
	 },
	 -- :: visualization
	 visuals = {
	    meshes.Joint,
	    meshes.segmentA,
	    meshes.massA,
	 },
      },
      -- ------------------------------------
      -- :: Frame::Segment B
      {
	 -- :: tree identification
	 name = "SegmentB",
	 parent = "SegmentA",
	 -- :: dynamics
	 body = dynamics.segmentB,
	 -- :: kinematics
	 joint_frame = {
	    r = { 0, 0.0, -l_1 }, 
	 },
	 joint = { 
	    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0} -- 
	 },
	 -- :: visualization
	 visuals = {
	    meshes.Joint,
	    meshes.segmentB,
	    meshes.massB,
	 },
      },
      {
	 -- :: tree identification
	 name = "end",
	 parent = "SegmentB",
	 joint_frame = {
	    r = { 0, 0.0, -l_2 }, 
	 },
      },
      
   }
}

-- ||-----------------------------------------------------------------||

return model
