$fn = 30;

use <MCAD/nuts_and_bolts.scad>

wall = 3;
internal_sz_z = 19.9;
sz_x = 20;
top_sz_y = 50;
bottom_sz_y = 20;
down_sz_z = 20;
bolt_hole_off = 10;
bolt_body_d = 4;
bolt_head_d = 6;
bolt_body_sz_z = 1.5;

  difference() {
    union() {
      translate([0, 0, internal_sz_z/2]) {
        cube([sz_x, top_sz_y,wall]);
      }
      translate([0, 0, -internal_sz_z/2-wall*2]) {
        cube([sz_x, bottom_sz_y, wall*2]);
      }
      rotate([0, 90, 0]) {
        cylinder(d=internal_sz_z+wall*2, h=sz_x);
      }
//      translate([0, bottom_sz_y, -internal_sz_z/2-down_sz_z]) {
//        cube([sz_x, wall, down_sz_z]);
//      }
//    translate([sz_x/2, bolt_hole_off, -internal_sz_z/2-wall-bolt_body_sz_z]) {
//      cylinder(d=bolt_body_d, h=bolt_body_sz_z);
//    }
    }
    rotate([0, 90, 0]) {
      cylinder(d=internal_sz_z, h=sz_x);
    }
    translate([0, 0, -internal_sz_z/2]) {
      cube([sz_x, bottom_sz_y, internal_sz_z]);
    }
    translate([sz_x/2, bolt_hole_off, -internal_sz_z/2-wall*2]) {
      cylinder(d=bolt_body_d, h=bolt_body_sz_z+wall*2);
      translate([0, 0, wall]) {
        nutHole(4);
      }
    }
  }
