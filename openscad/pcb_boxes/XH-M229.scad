$fn = 30;

wall = 2.5;
pcb_sz_x = 128.4;
pcb_sz_y = 48;
pcb_under_sz_z = 8;
bar_sz_x = 153;
bar_sz_y = 7.4;
bar_dist_y = pcb_sz_y - 2 * bar_sz_y;

hole_dist_y = 42;
hole_dist_x = 122.2;

attach_down_hole_dist_z = 11;

//union() {
difference() {
	for(yp=[0, 1]) {
		translate([wall, yp*(bar_sz_y+bar_dist_y), 0]) {		
			cube([bar_sz_x, bar_sz_y, pcb_under_sz_z]);
			if(yp==0) {
				for(xp=[0, 1]) {
					translate([xp*bar_sz_x-wall, 0, 0]) {
						hull() {
							cube([wall, 2*bar_sz_y+bar_dist_y, pcb_under_sz_z]);
							translate([-1+xp*1, 0, -16]) {
								cube([wall+1, bar_sz_y, 16+pcb_under_sz_z]);
							}
						}
					}
				}
			}
		}
	}
	translate([0, (bar_sz_y+bar_dist_y+2*bar_sz_y-pcb_sz_y)/2, 0]) {
		translate([wall+(bar_sz_x-pcb_sz_x)/2, 0, -25]) {
			for(xp=[0, hole_dist_x]) {
				for(yp=[0, hole_dist_y]) {
					translate([xp, yp, 0]) {
						cylinder(d=2.9, h=50);
					}
				}
			}
		}
		translate([-bar_sz_x/2, 0, -attach_down_hole_dist_z]) {
			rotate([0, 90, 0]) {
				cylinder(d=3.6, h=bar_sz_x*2);
			}
		}
	}
}
