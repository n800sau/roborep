initial_height = 7;
cube_count = 2;
cube_set_count = 5;
height_step = 1;
cube_width = 20;
cube_length = 10;
$fn = 30;
text_font = "Open Sans:style=Bold";
hole_height = initial_height+(cube_count)*height_step;
for(j=[0:cube_set_count-1]) {
	translate([j*(cube_length+2), 0, 0]) {
		for(i = [0:cube_count-1]) {
			translate([0, i*(cube_width+2), 0]) {
				difference() {
					cube([cube_length, cube_width, initial_height+i*height_step]);
					translate([cube_length/2, 0, 0]) {
						translate([0, cube_width/2, hole_height/2-1]) {
							cylinder(hole_height+2, r=2, center=true);
						}
						translate([0, cube_width/4-1, initial_height+i*height_step-3]) {
							linear_extrude(7, convexity=3) {
								text(str(initial_height+i*height_step), size=5, valign="center", halign="center",
										font=text_font, $fn=36);
							}
						}
					}
				}
			}
		}
	}
}
