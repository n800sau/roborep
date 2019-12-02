use <pcb_box_7x5.scad>

pcb_box(
	make_type="bottom",
	wall = 2,
	pcb_sz_x = 54,
	pcb_sz_y = 28.3,
	pcb_sz_z = 3,
	pcb_under_sz_z = 4,
	pcb_wall_gap = 6,
	side_sz_z = 10,
	screw_d = 2,
	screw_holes = [
		[2.5, 2],
		[-2.5, -2],
		[-2.5, 2],
		[2.5, -2],
	],
	leg_sz_z = 0,
	leg_pos = [],
	leg_d = 5,
	corner_screw_d=3,
  add_attachment=true
);
