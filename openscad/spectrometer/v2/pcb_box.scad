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

	for(p=screw_holes) {
		translate([wall+2*pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]), wall+2*pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), 0]) {
			difference() {
				cylinder(d=screw_d+2*wall, h=side_sz_z-pcb_sz_z, center=true);
				cylinder(d=screw_d, h=side_sz_z-pcb_sz_z, center=true);
			}
		}
	}

}

pcb_box(screw_holes=[
	[6, 6],
	[6, -6],
	[-6, -6],
	[-6, 6],
	[15, 25],
]);
