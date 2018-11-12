-- ||-----------------------------------------------------------------||
--                      ...::: Exercice 3 :::...
-- ||-----------------------------------------------------------------||

-- ::External dependencies:: 
math = require('math')

-- ||-----------------------------------------------------------------||
-- || System Parameters |

-- ::INFO:: add the system parameter from assignment 3.3 below
-- beware to stick to the SI-unit system (!!)

-- ::rod lengths::
local l_1 = 0.40  -- [m]  <--- add value here

-- ::sphere radian::
local r_1 = 0.08  -- [m]  <--- add value here

-- ||-----------------------------------------------------------------||


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
      dimensions = { 0.05, 0.05, l_1 }, -- [m] <-- add appropriate values here
      mesh_center = { 0.0,0, -l_1/2.0 }, -- [m] <-- add appropriate values here
   },
   massA = {
      name = "MassA",
      color = { 0.0, 0.0, 1.0}, 
      src = "meshes/unit_sphere_medres.obj",
      dimensions = { r_1*2.0, r_1*2.0, r_1*2.0 }, -- [m] <-- add appropriate values here
      mesh_center = { 0, 0.0, -l_1 },             -- [m] <-- add appropriate values here
   },
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
	    r = { 0, 0 , l_1*1.5 }, -- [m]
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
	 -- :: kinematics
	 joint_frame = {
	    r = { 0, 0 , 0.0 },    -- [m] <-- add appropriate values here
	 },
	 -- :: visualization
	 visuals = {
	    meshes.Joint,
	    meshes.segmentA,
	    meshes.massA,
	 },
      }
   }
}

-- ||-----------------------------------------------------------------||

return model