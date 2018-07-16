include <bb_box_params.scad>;
use <nuts_and_bolts.scad>;

bblid_thickness = 3;
cyl_height = 3;

ll_length = 7+bblid_thickness;
ll_width = 20;
ll_thickness = 6;

//44 betwen holes

module lidlock(cyl_r, with_hole) {
	difference() {
		union() {
			translate([0, 0, -ll_thickness]) {
				cube([ll_width, ll_length, ll_thickness], center=true);
			}
			if(!with_hole) {
				translate([0, -ll_length/2 + 6, -ll_thickness/2+cyl_height/2]) {
					sphere(d=cyl_r*2, center=true);
        }
				translate([0, -ll_length/2 + 6, -1-ll_thickness/2+cyl_height/2]) {
					cylinder(r=cyl_r, h=cyl_height, center=true);
				}
			}
		}
		if(with_hole) {
			translate([0, -ll_length/2 + 6, -ll_thickness/2+cyl_height/2]) {
				cylinder(r=cyl_r, h=20, center=true);
			}
			translate([0, 0, -5]) {
				scale([1.2, 1.2, 2]) {
					mNut(center=true);
				}
			}
		}
	}
}

module RoundedBBLid() {

	full_width = bb_width + power_side_width +
    other_side_width - bb_curve_corner + wall_thickness+1;
  echo("Full Width:", full_width);
	full_height = bb_height-bb_curve_corner+wall_thickness*2;
	minkowski() {
		cube([full_width+ll_thickness*2, full_height, bblid_thickness],center=true);
		cylinder(r=bb_curve_corner/2, height=bblid_thickness);
	}

	translate([full_width/2+2,-bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, -90, 0]) {
        lidlock(hole3m_radius-0.3, false);
      }
		}
	}

	translate([full_width/2+2, bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, -90, 0]) {
        lidlock(hole3m_radius-0.3, false);
      }
		}
	}

	translate([-(full_width/2+2), -bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, 90, 0]) {
        lidlock(hole3m_radius, true);
      }
    }
	}

	translate([-(full_width/2+2), bb_height/4, ll_length/2+bblid_thickness/2]) {
		rotate([90, 0, 0]) {
      rotate([0, 90, 0]) {
        lidlock(hole3m_radius, true);
      }
    }
	}

}

RoundedBBLid();
