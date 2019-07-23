$fn = 20;


module pcb_box(
	wall = 2,
	pcb_sz_x = 30,
	pcb_sz_y = 50,
	pcb_sz_z = 1,
	pcb_under_sz_z = 4,
	pcb_wall_gap = 4,
	side_sz_z = 10,
	screw_d = 2,
	screw_holes = [],
	leg_sz_z = 0,
	leg_pos = [],
	leg_d = 5,
  corner_screw_d=3
) {
	translate([wall+pcb_wall_gap, wall+pcb_wall_gap, wall+pcb_under_sz_z]) {
%		cube([pcb_sz_x, pcb_sz_y, pcb_sz_z]);
	}
	translate([wall, wall, 0]) {
		cube([pcb_sz_x+2*pcb_wall_gap, pcb_sz_y+2*pcb_wall_gap, wall]);
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
			translate([wall+2*pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]), wall+2*pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), -leg_sz_z/2]) {
				cylinder(d=leg_d, h=leg_sz_z, center=true);
			}
		}
	}

	for(p=screw_holes) {
		translate([wall+pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]),
				wall+pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), wall+pcb_under_sz_z/2]) {
			difference() {
				cylinder(d=screw_d+2*wall, h=pcb_under_sz_z, center=true);
				cylinder(d=screw_d, h=pcb_under_sz_z, center=true);
			}
		}
	}

	if(pcb_wall_gap > 0) {
		for(x=[0,1]) {
			for(y=[0,1]) {
				translate([wall+pcb_wall_gap/2+x*(pcb_sz_x+pcb_wall_gap), wall+pcb_wall_gap/2+y*(pcb_sz_y+pcb_wall_gap), side_sz_z/2]) {
					difference() {
						union() {
							cylinder(d=corner_screw_d+2*wall, h=side_sz_z, center=true);
              translate([0, 0, -side_sz_z/2]) {
                rotate([0, 0, (y==0 ? 90*x : 180-90*(x-1))]) {
                  linear_extrude(height=side_sz_z)
                  {
                      translate([-pcb_wall_gap/2, -pcb_wall_gap/2, 0]) {
                        polygon(points=[[0,0],[pcb_wall_gap,0],[0,pcb_wall_gap]], paths=[[0,1,2]]);
                      }
                  }
                }
              }
						}
						cylinder(d=corner_screw_d, h=side_sz_z, center=true);
					}
				}
			}
		}
	}

}

module pcb_box_7x5(leg_sz_z=0, leg_pos=[]) {
	pcb_box(
		pcb_sz_x = 70,
		pcb_sz_y = 50,
		pcb_sz_z = 2,
    pcb_wall_gap = 5,
    side_sz_z = 8,
		screw_holes=[
			[3, 3],
			[3, -3],
			[-3, -3],
			[-3, 3],
		],
		leg_sz_z=leg_sz_z,
		leg_pos=leg_pos
	);
}

pcb_box_7x5();
