$fn = 30;

wall = 3;

internal_sz = 69;
internal_sz_z = 79;

bottom_bar_length = internal_sz + 2 * wall;
bottom_bar_width = 18;
bottom_bar_height = wall;

bar_sz_z = 6;
bar_step_z_n = 3;
bar_step_z = (internal_sz_z-bar_sz_z) / bar_step_z_n;

bar_step_count = 7;

attachment_dist_z = 38;
attachment_d = 26.4;

hole_d_through = 4;
bolt_cap_d = 7.4;
bolt_cap_sz = 2.2;


hole_dist = 14;

echo("size=", (bottom_bar_length-1)/bar_step_count);

difference() {
	union() {
		for(xp=[0, 1]) {
			translate([xp*(wall+internal_sz+wall-bottom_bar_width), 0, 0]) {
				cube([bottom_bar_width, bottom_bar_length, bottom_bar_height]);
			}
			for(zp=[0:1:bar_step_z_n]) {
				translate([xp*(wall+internal_sz), 0, zp*bar_step_z]) {
					cube([wall, bottom_bar_length, bar_sz_z]);
				}
			}
		}
		for(yp=[0, 1]) {
			for(xpp=[0:1:bar_step_count]) {
				translate([xpp*((bottom_bar_length-1)/bar_step_count), yp*(wall+internal_sz), 0]) {
					cube([1, wall, internal_sz_z]);
				}
			}
		}
		
		for(yp=[0, 1]) {
			translate([0, yp*(wall+internal_sz+wall-bottom_bar_width), 0]) {
				cube([bottom_bar_length, bottom_bar_width, bottom_bar_height]);
			}
			for(zp=[0:1:bar_step_z_n]) {
				translate([0, yp*(wall+internal_sz), zp*bar_step_z]) {
					cube([bottom_bar_length, wall, bar_sz_z]);
				}
			}
		}
		
		for(xp=[0, 1]) {
			for(ypp=[0:1:bar_step_count]) {
				translate([xp*(wall+internal_sz), ypp*((bottom_bar_length-1)/bar_step_count), 0]) {
					cube([wall, 1, internal_sz_z]);
				}
			}
		}
		// attachment
		translate([bottom_bar_length/2, wall, attachment_dist_z]) {
			rotate([90, 0, 0]) {
				cylinder(d=attachment_d, h=wall);
			}
		}
	}
	// attachment holes
	translate([bottom_bar_length/2, wall, attachment_dist_z]) {
		for(p=[-1,1]) {
			translate([p*hole_dist/2, 0, 0]) {
				rotate([90, 0, 0]) {
					cylinder(d2=hole_d_through, d1=bolt_cap_d, h=bolt_cap_sz);
					cylinder(d=hole_d_through, h=4*wall);
				}
			}
		}
	}	
}	
