include <cnc_params.scad>


module pipette_mockup() {
	translate([0, 0, 81]) {
		translate([0, 0, 28]) {
			translate([-22, 0, 101]) {
				cube([39+22, 25, 32]);
			}
			hull() {
				translate([0, 0, 100]) {
					cube([33.2, 25, 1]);
				}
				cube([pipette_bottom_holder_sz_x, pipette_bottom_holder_sz_y, 1]);
			}
		}
		translate([pipette_wall, pipette_wall, 0]) {
			hull() {
				translate([0, 0, 28]) {
					cube([pipette_bottom_holder_sz_x-2*pipette_wall, pipette_bottom_holder_sz_y-2*pipette_wall, 1]);
				}
				cube([pipette_bottom_holder_sz_x-2*pipette_wall, pipette_bottom_holder_sz_y-2*pipette_wall, 1]);
			}
		}
	}
	translate([pipette_wall+4.5+6, pipette_wall+4.5+6, 0]) {
		cylinder(d2=12, d1=8, h=81);
	}
}

pipette_mockup();
