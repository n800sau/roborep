include <cnc_params.scad>
$fn = 40;

module duratech_maket() {
	translate([0, 0, -drill_bit_sz_z/2]) {
		cylinder(d=drill_bit_d, h=drill_bit_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z/2]) {
		cylinder(d1=metal_cone_bottom_d, d2=metal_thing_d, h=metal_cone_bottom_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z/2]) {
		cylinder(d=metal_thing_d, h=metal_thing_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z/2]) {
		cylinder(d=bottom_d, h=bottom_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z+above_bottom_sz_z/2]) {
		cylinder(d1=bottom_d, d2=above_bottom_d, h=above_bottom_sz_z, center=true);
	}
	translate([above_bottom_box_sz_x/2, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z+above_bottom_box_sz_z/2]) {
		cube([above_bottom_box_sz_x, above_bottom_box_sz_y, above_bottom_box_sz_z], center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z+above_bottom_sz_z+widest_level_sz_z/2]) {
		cylinder(d1=above_bottom_d, d2=widest_level_d, h=widest_level_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z+
				above_bottom_sz_z+widest_level_sz_z+above_widest_level_sz_z/2]) {
		cylinder(d1=widest_level_d, d2=above_widest_level_d, h=above_widest_level_sz_z, center=true);
	}
	translate([0, 0, metal_cone_bottom_sz_z+metal_thing_sz_z+bottom_sz_z+
				above_bottom_sz_z+widest_level_sz_z+above_widest_level_sz_z+top_sz_z/2]) {
		cylinder(d1=above_widest_level_d, d2=top_d, h=top_sz_z, center=true);
	}
}

duratech_maket();
