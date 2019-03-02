use <MCAD/libtriangles.scad>

$fn = 50;
wall = 2;
side_pad = 5;
back_pad = 4;
power_hole_d = 8.8;
bottom_sz_x = power_hole_d+side_pad*2;
bottom_sz_y = 15;
hole_d = 3.2;
ear_sz_x = hole_d + 5;
ear_sz_y = hole_d + 5;

translate([-wall, 0, 0]) {
  difference() {
    cube([power_hole_d+side_pad*2+wall*2, wall, power_hole_d+side_pad*2+wall]);
    translate([power_hole_d/2+side_pad+wall, bottom_sz_y, power_hole_d/2+side_pad+wall]) {
      rotate([90, 0, 0]) {
        cylinder(d=power_hole_d, h=30);
      }
    }
  }
}
translate([0, 0, 0]) {
  cube([bottom_sz_x, power_hole_d+back_pad*2, wall]);
  translate([-ear_sz_x, bottom_sz_y-wall-ear_sz_y/2, 0]) {
  //  union() {
    difference() {
      translate([ear_sz_x/2, 0, 0]) {
        cube([bottom_sz_x+ear_sz_x, ear_sz_y, wall]);
        translate([0, ear_sz_y/2, 0]) {
          cylinder(d=ear_sz_x, h=wall);
        }
        translate([bottom_sz_x+ear_sz_x, ear_sz_y/2, 0]) {
          cylinder(d=ear_sz_x, h=wall);
        }
      }
      translate([ear_sz_x/2, ear_sz_y/2, -10]) {
        cylinder(d=hole_d, h=20);
        translate([bottom_sz_x+ear_sz_x, 0, 0]) {
          cylinder(d=hole_d, h=20);
        }
      }
    }
  }
}
for(x=[0, bottom_sz_x+wall]) {
  translate([x, wall, 0]) {
    rotate([0, 0, 90]) {
      rightprism(bottom_sz_x/2, wall, power_hole_d+side_pad*2+wall);
    }
  }
}

