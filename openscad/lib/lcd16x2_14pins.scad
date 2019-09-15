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
  wall_h = 15,
  extra_sz_x = 6,
  extra_sz_y = 6,
  hole_d = 2.9,
  stand_h = 5,
  stand_d = 6,
  wall = 2
) {
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
  extra_sz_x = 6,
  extra_sz_y = 6,
  hole_d = 2.9,
//  stand_h = 15-5-wall-,
  stand_d = 6,
  wall = 2
) {
  difference() {
    union() {
      cube([lcd_pcb_l+extra_sz_x+2*wall, lcd_pcb_w+extra_sz_y+2*wall, wall], center=true);
      translate([0, 0, -stand_h/2]) {
        lcd_holes(stand_h, stand_d, center=true);
      }
    }
//    translate([0, 0, stand_h/2]) {
//      lcd_holes(h=stand_h, d=hole_d, center=true);
//    }
//    lcd_box_hole(h=stand_h, center=true);
  }
}

//lcd_all_holes(center=true);
lcd_top();
translate([0, 0, 16]) {
//  lcd_bottom();
}
