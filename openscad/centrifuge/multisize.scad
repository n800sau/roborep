base_d = 85;
base_h = 20;

box_len = base_d;
box_width = 23;

rotor_d = 30;
rotor_h = 5;

hole_dist = 21;
hole_d = 2.5;

insert_width = 2.1;
insert_len = 34;
insert_height = base_h;

$fn = 200;

module centry() {
//	union() {
	difference() {
		// base body
		cylinder(d = base_d, h = base_h);


		translate([0, 0, rotor_h]) {
			cylinder(d = rotor_d, h = base_h);
    }

		for(i=[0: 6]) {
			// Rotor mount holes
			rotate([0, 0, i*360/6]) {
				translate([0, hole_dist/2, -0.1])
					cylinder(d = hole_d, h = 6.1);
			}
		}
		for(i=[0: 4]) {
			rotate([0, 0, i*360/4]) {
				translate([-box_width/2, rotor_d/2+3, rotor_h]) {
					cube([box_width, box_len, base_h]);
        }
				translate([-insert_len/2, rotor_d/2+10, rotor_h]) {
					cube([insert_len+1, insert_width+1, base_h]);
        }
			}
		}
    tick_marker_hole();
	}
}

module tick_marker_hole() {
  rotate([0, 0, 45]) {
    translate([base_d/2-11, 0, 2.5])
//        cylinder(d=6.1, h=5);
      cube([6, 6, 5], center=true);
  }
}

module insert_plate(d) {
	difference()  {
		cube([insert_len, insert_height+4, insert_width], center=true);
		translate([0, -2, 0])
			cylinder(d=d, h=insert_width*2, center=true);
		translate([0, insert_height/2-1, 0])
			cube([insert_len/2, 2, insert_width*2], center=true);
	}
}


centry();

//insert_plate(5);
//insert_plate(8);
//insert_plate(11.5);
//rotate([90, 90, 90])
//import("12mm.stl");
//insert_plate(15);

  