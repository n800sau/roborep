use <../lib/pcb_led_cvf.scad>

$fn = 40;

border_sz = 6;

wall = 2;

tft_module_sz_x = 80;
tft_module_sz_y = 70;
tft_module_sz_z = 3;

tft_hole_sz_x = 70;
tft_hole_sz_y = 60;

tft_field_sz_left = 10;
tft_field_sz_top = 10;
tft_field_sz_right = tft_module_sz_x - tft_hole_sz_x - tft_field_sz_left;
tft_field_sz_bottom = tft_module_sz_y - tft_hole_sz_y - tft_field_sz_top;

cable_space_module_sz_left = 30;
cable_space_sz_x = 30;

x_hole_offset = 30;
y_hole_offset = 30;

hole_d = 2.9;
hole_d_through = 3.6;

controller_pcb_sz_x = 40;
controller_pcb_sz_y = 30;
controller_pcb_sz_z = 2;

controller_pcb_below_sz_z = 3;
controller_pcb_above_sz_z = 10;


module top_cover() {
	union() {
		union() {
			// first lever - face
			cube([border_sz + tft_field_sz_left, tft_module_sz_y+2*border_sz, wall]);
			translate([border_sz + tft_module_sz_x - tft_field_sz_right, 0, 0]) {
				cube([tft_field_sz_right + border_sz, tft_module_sz_y+2*border_sz, wall]);
			}
			cube([tft_module_sz_x+2*border_sz, border_sz + tft_field_sz_top, wall]);
			translate([0, border_sz + tft_module_sz_y - tft_field_sz_bottom, 0, 0]) {
				cube([tft_module_sz_x+2*border_sz, tft_field_sz_bottom + border_sz, wall]);
			}
			// next lever - border around
			translate([0, 0, wall]) {
				difference() {
					union() {
						cube([border_sz, tft_module_sz_y+2*border_sz, tft_module_sz_z]);
						translate([border_sz + tft_module_sz_x, 0, 0]) {
							cube([border_sz, tft_module_sz_y+2*border_sz, tft_module_sz_z]);
						}
						cube([tft_module_sz_x+2*border_sz, border_sz, tft_module_sz_z]);
						translate([0, border_sz + tft_module_sz_y, 0]) {
							cube([tft_module_sz_x+2*border_sz, border_sz, tft_module_sz_z]);
						}
					}
					// extraction for cable
					translate([cable_space_module_sz_left, border_sz + tft_module_sz_y, 0]) {
						cube([cable_space_sz_x, border_sz, tft_module_sz_z]);
					}
				}
			}
		}
		// border holes along x
		for(x=[0, 0.5, 1]) {
			for(y=[0, 1]) {
				translate([x_hole_offset+x*(tft_module_sz_x+2*border_sz-2*x_hole_offset), border_sz/2+y*(tft_module_sz_y+border_sz), -25]) {
					cylinder(d=hole_d_through, h=50, center=true);
				}
			}
		}
		// border holes along y
		for(y=[0, 1]) {
			for(x=[0, 1]) {
				translate([border_sz/2+x*(tft_module_sz_x+border_sz), y_hole_offset+y*(tft_module_sz_y+2*border_sz-2*y_hole_offset), -25]) {
					cylinder(d=hole_d_through, h=50, center=true);
				}
			}
		}
	}
}

module controller_holder() {
	translate([border_sz+tft_module_sz_x/2-controller_pcb_sz_x/2-1.5, border_sz+tft_module_sz_y/2-controller_pcb_sz_y/2-1.5, 0]) {
		pcb_led_cvf(
			pcb_length=controller_pcb_sz_x,
			pcb_width=controller_pcb_sz_y,
			pcb_height=controller_pcb_sz_z,
			pcb_bottom_margin=controller_pcb_under_sz_z,
			add_table=false
		);
	}
	for(x=[0, 0.5, 1]) {
		translate([x_hole_offset+x*(tft_module_sz_x+2*border_sz-2*x_hole_offset)-5, 0, 0]) {
			cube([10, tft_module_sz_y, wall]);
		}
	}
	for(y=[0, 1]) {
		translate([0, y_hole_offset+y*(tft_module_sz_y+2*border_sz-2*y_hole_offset)-5, 0, 0]) {
			cube([tft_module_sz_x, 10, wall]);
		}
	}
}

//top_cover();
translate([0, 0, 10]) {
	controller_holder();
}

