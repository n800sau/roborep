type="single"; // single, double (sided)

$fn = 30;

module pcb_box(
	// make_type - bottom, top, middle
	make_type="bottom",
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
	corner_screw_d=3,
  add_attachment=true
) {

	// mode - 'stand' or 'hole'
	module corner_holes(mode) {
			for(x=[0,1]) {
				for(y=[0,1]) {
					translate([wall+pcb_wall_gap/2+x*(pcb_sz_x+pcb_wall_gap), wall+pcb_wall_gap/2+y*(pcb_sz_y+pcb_wall_gap), side_sz_z/2]) {
						if(mode =="stand") {
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
						} else {
							translate([0, 0, wall]) {
								cylinder(d=corner_screw_d, h=side_sz_z+2*wall, center=true);
							}
						}
					}
				}
			}
	}

  module legs(
        leg_sz_z = 0,
        leg_pos = [],
        leg_d = 5
      ) {
    for(p=leg_pos) {
      translate([wall+2*pcb_wall_gap+(p[0]>=0 ? p[0] : pcb_sz_x+p[0]), wall+2*pcb_wall_gap+(p[1]>=0 ? p[1] : pcb_sz_y+p[1]), -leg_sz_z/2]) {
        cylinder(d=leg_d, h=leg_sz_z, center=true);
      }
    }
  }

	difference() {
		union() {
      if(make_type != "middle") {
        translate([wall, wall, (make_type == "bottom" ? 0 : side_sz_z-wall)]) {
 		  cube([pcb_sz_x+2*pcb_wall_gap, pcb_sz_y+2*pcb_wall_gap, wall]);
        }
      }
			// walls along x
			for(y=[0, pcb_sz_y+2*pcb_wall_gap+wall]) {
				translate([0, y, 0]) {
					cube([pcb_sz_x+2*pcb_wall_gap+2*wall, wall, side_sz_z]);
				}
			}
			// walls along y
			for(x=[0, pcb_sz_x+2*pcb_wall_gap+wall]) {
				translate([x, 0, 0]) {
					cube([wall, pcb_sz_y+2*pcb_wall_gap+2*wall, side_sz_z]);
				}
			}

			// corner stand
			if(pcb_wall_gap > 0) {
				corner_holes("stand");
			}
		}
		// corner holes
		if(pcb_wall_gap > 0) {
			corner_holes("hole");
		}
	}

	if(make_type == "bottom") {

		// pcb placement shown
		translate([wall+pcb_wall_gap, wall+pcb_wall_gap, wall+pcb_under_sz_z]) {
			%cube([pcb_sz_x, pcb_sz_y, pcb_sz_z]);
		}

		if(leg_sz_z>0) {
      legs(leg_sz_z=leg_sz_z, leg_pos=leg_pos, leg_d=leg_d);
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

    if(add_attachment) {
      translate([(pcb_sz_x+2*pcb_wall_gap+2*wall)/2, (pcb_sz_y+2*pcb_wall_gap+2*wall)/2, wall/2]) {
        for(x=[-1,1]) {
          for(y=[-1,1]) {
            translate([x*pcb_sz_x/4, y*(pcb_sz_y+2*pcb_wall_gap+2*wall)/2, 0]) {
              rotate([0, 0, x*180-y*90]) {
                attachment(l=20, w=10, h=2);
              }
            }
          }
        }
      }
    }

	} else if(make_type == "top") {
	}
}

module attachment(l=20, w=10, h=2, hole_d=3.5) {
  translate([-w/4, 0, 0]) {
    difference() {
      union() {
        cube([l-w/2, w, h], center=true);
        translate([w-w/4, 0, 0]) {
          cylinder(d=w, h=h, center=true);
        }
      }
      translate([w-w/4, 0, 0]) {
        cylinder(d=hole_d, h=h, center=true);
      }
    }
  }
}

module pcb_box_SBT5333(
    make_type="bottom", side_sz_z = 9, leg_sz_z=3, leg_pos=[], corner_screw_d=3
  ) {
	pcb_box(
		make_type=make_type,
		pcb_sz_x = 60.4,
		pcb_sz_y = 43,
		pcb_sz_z = 1.5,
		pcb_wall_gap = 6,
		side_sz_z = side_sz_z,
		screw_holes=[
			[3.5, 3.5],
			[3.5, -3.5],
			[-3.5, -3.5],
			[-3.5, 3.5],
		],
		leg_sz_z=leg_sz_z,
		leg_pos=leg_pos,
		pcb_under_sz_z = 5,
    	corner_screw_d=corner_screw_d,
		screw_d = 3.2
	);
}

//pcb_box_SBT5333(
//	make_type="top",
//	side_sz_z =25
//);

pcb_box_SBT5333(
	make_type="bottom"
);
