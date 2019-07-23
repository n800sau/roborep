$fn = 20;


module pcb_box(
	wall = 2,
	pcb_sz_x = 30,
	pcb_sz_y = 50,
	pcb_sz_z = 2,
	pcb_wall_gap = 1,
	side_sz_z = 10,
	screw_d = 3,
	screw_holes = [],
	leg_sz_z = 0,
	leg_pos = [],
	leg_d = 5
) {
	translate([wall, wall, 0]) {
		cube([pcb_sz_x+2*pcb_wall_gap, pcb_wall_gap*2+pcb_sz_y, wall]);
	}
	for(y=[0, pcb_sz_y+2*pcb_wall_gap+wall]) {
		translate([0, y, 0]) {
			cube([pcb_sz_x+2*pcb_wall_gap+2*wall, wall, side_sz_z]);
		}
	}

	for(x=[0, pcb_sz_x+2*pcb_wall_gap+wall]) {
		translate([x, 0, 0]) {
			cube([wall, pcb_sz_y+2*pcb_wall_gap+2*wall, side_sz_z]);
		}
	}

	if(leg_sz_z>0) {
		for(p=leg_pos) {
			translate([wall+2*pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]), wall+2*pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), -leg_sz_z-wall]) {
				cylinder(d=leg_d, h=leg_sz_z, center=true);
			}
		}
	}

	for(p=screw_holes) {
		translate([wall+2*pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]), wall+2*pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), 0]) {
			difference() {
				cylinder(d=screw_d+2*wall, h=side_sz_z-pcb_sz_z, center=true);
				cylinder(d=screw_d, h=side_sz_z-pcb_sz_z, center=true);
			}
		}
	}

}

module pcb_box_7x6(leg_sz_z=0, leg_pos=[]) {
	pcb_box(
		pcb_sz_x = 70,
		pcb_sz_y = 60,
		pcb_sz_z = 2,
		screw_holes=[
			[6, 6],
			[6, -6],
			[-6, -6],
			[-6, 6],
		],
		leg_sz_z=leg_sz_z,
		leg_pos=leg_pos
	);
}

pcb_box_7x6();
