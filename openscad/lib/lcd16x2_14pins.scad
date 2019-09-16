use <MCAD/boxes.scad>

$fn = 50;

lcd_pcb_l = 85;
lcd_pcb_w = 30;


// 16x02 lcd
lcd_l = 71;
lcd_w = 24;

interhole_l = 82;
interhole_w = 23.5;

lcd_holes_shift_y = 0;

pin_hole_sz_x = 8;
pin_hole_sz_y = 18.6;
pin_hole_off_x = 0;

module lcd_box_hole(h, center=false) {
	translate([center ? 0 : lcd_pcb_l/2, center ? 0 : lcd_pcb_w/2, 0]) {
		cube([lcd_l, lcd_w, h], center=true);
	}
}

module lcd_holes(h, d, center=false) {
	translate([center ? 0 : lcd_pcb_l/2, center ? 0 : lcd_pcb_w/2, 0]) {
		for(xsgn=[-1,1]) {
				for(ysgn=[-1,1]) {
				translate([xsgn*interhole_l/2, ysgn*interhole_w/2, 0]) {
					cylinder(d=d, h=h, center=true);
				}
			}
		}
	}
}

module lcd_all_holes(h=10, d=3.5, center=false) {
	lcd_holes(d=d, h=h, center=center);
	translate([0, lcd_holes_shift_y, 0]) {
		lcd_box_hole(h=h, center=center);
	}
}

module lcd_top(
	extra_sz_x = 4,
	extra_sz_y = 4,
	hole_d = 2.9,
	stand_h = 5,
	stand_d = 6,
  pcb_h = 1.6,
  outer_stand_h = 6,
	wall = 2
) {
  wall_h = wall + stand_h + pcb_h + outer_stand_h;
	difference() {
		union() {
			translate([0, 0, (wall_h-wall)/2]) {
				difference() {
					roundedBox([lcd_pcb_l+extra_sz_x+2*wall, lcd_pcb_w+extra_sz_y+2*wall, wall_h], 2, true);
					roundedBox([lcd_pcb_l+extra_sz_x, lcd_pcb_w+extra_sz_y, wall_h], 2, true);
				}
			}
			cube([lcd_pcb_l+extra_sz_x, lcd_pcb_w+extra_sz_y, wall], center=true);
			translate([0, 0, stand_h/2]) {
				lcd_holes(stand_h, stand_d, center=true);
			}
		}
		translate([0, 0, stand_h/2]) {
			lcd_holes(h=stand_h, d=hole_d, center=true);
		}
		lcd_box_hole(h=stand_h, center=true);
	}
}

module lcd_bottom(
	wall_h = 15,
	extra_sz_x = 4,
	extra_sz_y = 4,
	hole_d_through = 3.4,
	wall = 2
) {
	difference() {
		union() {
      roundedBox([lcd_pcb_l+extra_sz_x+2*wall, lcd_pcb_w+extra_sz_y+2*wall, wall], 2, true);
      translate([0, 0, 11]) {
        rotate([-90, 0, 0]) {
          go_pro_connector();
        }
      }
		}
		translate([0, 0, 0]) {
			lcd_holes(h=20, d=hole_d_through, center=true);
		}
    // hole for connectors
		translate([(lcd_pcb_l-pin_hole_sz_x-0.4)/2-pin_hole_off_x, 0, 0]) {
			cube([pin_hole_sz_x+0.4, pin_hole_sz_y+0.4, 20], center=true);
		}
	}
}

module go_pro_connector() {
  include <gopro_mounts_mooncactus.scad>
  gopro_primary="triple";
  gopro_secondary_what="none";
  gopro_ext_len=20;
}

//lcd_top();
translate([0, 0, 16]) {
	lcd_bottom();
}
