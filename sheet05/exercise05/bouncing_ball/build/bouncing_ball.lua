
math = require('math')


meshes = {
   
   massA = {
      name = "MassA",
      color = { 0.0, 0.0, 1.0}, 
      src = "meshes/unit_sphere_medres.obj",
      dimensions = { 1,1,1 }, -- [m] <-- add appropriate values here
      mesh_center = { 0, 0.0, 0 },             -- [m] <-- add appropriate values here
   },
}


model = {

   configuration = {
      axis_front = { 1, 0, 0 },
      axis_up    = { 0, 0, 1 },
      axis_right = { 0, -1, 0 },
      rotation_order = { 2, 1, 0},
   },

  
   frames = {
   
      {

	 name = "Ball",
	 parent = "ROOT",
	 -- :: kinematics
	 joint_frame = {
	    r = { 0, 0 , 0.0 },   
	 },

	 visuals = {
	    meshes.massA,
	 },
      }
   }
}

-- ||-----------------------------------------------------------------||

return model
