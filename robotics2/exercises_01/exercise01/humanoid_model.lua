
-- we assume our robot to be rather cubical
function get_box_inertia (mass, width, height, depth)
    local valx, valy, valz
    valx = 1.0/12.0 * mass * (height*height + depth*depth)
    valy = 1.0/12.0 * mass * (width*width + height*height)
    valz = 1.0/12.0 * mass * (width*width + depth*depth)
    return {
        {valx,  0.0,  0.0},
        { 0.0, valy,  0.0},
        { 0.0,  0.0, valz}
    }
end

--- for the base_link .... probably it does not nead any physical values?
function get_sphere_inertia (mass, r)
    local valx
    val = 2.0/5.0 * mass * r*r
    return {
        {val,  0.0,  0.0},
        { 0.0, val,  0.0},
        { 0.0,  0.0, val}
    }
end

-- define model variables here for each segment
pelvis_w = 0.5
pelvis_d = 0.5
pelvis_h = 0.5
pelvis_m = 10.0

base_r = pelvis_w/2.0 --added by me
base_m = 5.0

joint_r = 0.2

-- fill in for thigh, shank, foot
thigh_w = 0.05
thigh_d = 0.2
thigh_h = 0.5
thigh_m = 1.0

shank_w = thigh_w
shank_d = thigh_d
shank_h = thigh_h
shank_m = thigh_m

foot_w = shank_w*3
foot_d = shank_d*3
foot_h = 0.04
foot_m = 1.0


-- add other bodies
pelvis    = { mass = pelvis_m, com = {0,0,pelvis_h/2.0},   inertia = get_box_inertia(pelvis_m,pelvis_w,pelvis_h,pelvis_d) }

base_link = { mass = base_m,   com = {0,0,0},   inertia = get_sphere_inertia(base_m,base_r) } 

thigh = { mass = thigh_m,   com = {0,0,-thigh_h/2.0},   inertia = get_box_inertia(thigh_m,thigh_w,thigh_h,thigh_d) }

shank = { mass = shank_m,   com = {0,0,-shank_h/2.0},   inertia = get_box_inertia(shank_m,shank_w,shank_h,shank_d) }

foot  = { mass = foot_m,   com = {0,0,-foot_h/2.0},   inertia = get_box_inertia(foot_m,foot_w,foot_h,foot_d) }
 

-- add more meshes
meshes = {
  Pelvis = {
    name = "Pelvis",
    dimensions = { pelvis_w, pelvis_d, pelvis_h }, 
    color = { 0.8, 0.8, 0.2},
	-- please explain how the mesh_center-atribute is linked together with its mesh 
    mesh_center = { 0, 0, pelvis_h/2.0}, 
    src = "meshes/unit_cube.obj",
  },  
  Base = {
    name = "Base",
    dimensions = { pelvis_d, pelvis_w, pelvis_h/4.0 },
    color = { 0.8, 0.8, 0.2}, 
    mesh_center = { 0, 0, 0}, 
    src = "meshes/unit_sphere_medres.obj",
  },
  Joint = {
    name = "Joint",
    dimensions = {joint_r, joint_r, joint_r},
    color = { 0, 1, 0},
    mesh_center = { 0, 0, 0 },
    src = "meshes/unit_sphere_medres.obj",
  },
  Thigh = {
    name = "Thigh",
    dimensions = {thigh_d, thigh_w, thigh_h},
    color = { 0, 0, 1},
    mesh_center = { 0, 0, -thigh_h/2.0},
    src = "meshes/unit_cube.obj",
  },
  Shank = {
    name = "Shank",
    dimensions = {shank_d, shank_w, shank_h},
    color = { 0, 0, 1},
    mesh_center = { 0, 0, -shank_h/2.0 },
    src = "meshes/unit_cube.obj",
  },
  Foot = {
    name = "Foot",
    dimensions = {foot_d, foot_w, foot_h},
    color = { 0, 0, 1},
    mesh_center = { foot_d/4, 0, -foot_h/2.0 },
    src = "meshes/unit_cube.obj",
  },

  
}

-- these are the bodies your robot should have
bodies = {
	base_link = base_link,
    	pelvis = pelvis,
	thigh_right = thigh,
	shank_right = shank,
    	foot_right = foot,
	thigh_left = thigh,
	shank_left = shank,
    	foot_left = foot
}

joints = {
	floating_base = {--what does this "joint-matrix" mean?
		{ 0., 0., 0., 1., 0., 0.},
		{ 0., 0., 0., 0., 1., 0.},
		{ 0., 0., 0., 0., 0., 1.},
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
    -- hip joint and pelvis joint
	spherical_xyz = {
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
    -- ankle joint
	rotational_xy = {
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
    -- knee joint
	rotational_y = {
		{ 0., 1., 0., 0., 0., 0.}
	},
	fixed = {}
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
            		-- starting link of your robot
			name = "base_link",
			parent = "ROOT",
			body = bodies.base_link,
            		joint_frame = {
              			r = { 0.0, 0.0 , 0.0 },
            		},
			joint = joints.floating_base,
            		visuals = { meshes.Base}
		},
		{
			name = "pelvis",
			parent = "base_link",
			body = bodies.pelvis,
			joint = joints.spherical_xyz,
            		joint_frame = {
              			r = { 0.0, 0, 0.0 },
            		},
            		visuals = { meshes.Pelvis, meshes.Joint}
		},
		{
			name = "thigh_right",
			parent = "base_link",
			body = bodies.thigh_right,
			joint = joints.spherical_xyz,
            		joint_frame = {
              			r = { 0.0, -pelvis_w/2.0, 0.0 },
            		},
            		visuals = { meshes.Thigh, meshes.Joint}
		},
		{
			name = "shank_right",
			parent = "thigh_right",
			body = bodies.shank_right,
			joint = joints.rotational_y,
            		joint_frame = {
              			r = { 0.0, 0.0, -thigh_h },
            		},
            		visuals = { meshes.Shank, meshes.Joint}
		},
		{
			name = "foot_right",
			parent = "shank_right",
			body = bodies.foot_right,
			joint = joints.rotational_xy,
            		joint_frame = {
              			r = { 0.0, 0.0, -shank_h },
            		},
            		visuals = { meshes.Foot, meshes.Joint}
		},
		{
			name = "thigh_left",
			parent = "base_link",
			body = bodies.thigh_left,
			joint = joints.spherical_xyz,
            		joint_frame = {
              			r = { 0.0, pelvis_w/2.0, 0.0 },
            		},
            		visuals = { meshes.Thigh, meshes.Joint}
		},
		{
			name = "shank_left",
			parent = "thigh_left",
			body = bodies.shank_left,
			joint = joints.rotational_y,
            		joint_frame = {
              			r = { 0.0, 0.0, -thigh_h },
            		},
            		visuals = { meshes.Shank, meshes.Joint}
		},
		{
			name = "foot_left",
			parent = "shank_left",
			body = bodies.foot_left,
			joint = joints.rotational_xy,
            		joint_frame = {
              			r = { 0.0, 0.0, -shank_h },
            		},
            		visuals = { meshes.Foot, meshes.Joint}
		},
	}
}

return model
