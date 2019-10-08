
border_sz = 6;

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

module top_cover() {
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

top_cover();


