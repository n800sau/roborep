use <../lib/vent.scad>

$fn = 50;

wall = 2.5;

// pins side
pside_sz_x = 45;
pside_sz_y = 80;
pside_sz_z = 40;

hole_d = 4.6;

power_wire_d = 7.6;
vcc_wire_d = 5;

switch_sz_x = 19.4;
switch_sz_z = 13.6;

//hole dist ~ 67

echo("hole_dist=", pside_sz_y-6-7);

difference() {
  cube([pside_sz_x+wall, pside_sz_y+2*wall, pside_sz_z+2*wall], center=true);
  translate([-wall/2, 0, 0]) {
    cube([pside_sz_x, pside_sz_y, pside_sz_z], center=true);
  }
  cube([pside_sz_x+wall, pside_sz_y-10, pside_sz_z-10], center=true);
  // bolt hole bottom
  translate([-pside_sz_x/2+10, -pside_sz_y/2+7, -pside_sz_z/2]) {
    cylinder(d=hole_d, h=20, center=true);
  }
  // bolt hole top
  translate([-pside_sz_x/2+5, pside_sz_y/2-6, pside_sz_z/2]) {
    cylinder(d=hole_d, h=20, center=true);
  }
  translate([-50-pside_sz_x/2+5+14, 0, 0]) {
    %cube([100, pside_sz_y, pside_sz_z], center=true);
  }
  translate([0, -pside_sz_y/2+10, pside_sz_z/2]) {
    cylinder(d=power_wire_d, h=20, center=true);
    translate([-pside_sz_x/2, 0, 0]) {
      cube([pside_sz_x, power_wire_d, 20], center=true);
    }
  }
  translate([0, pside_sz_y/2-20, pside_sz_z/2]) {
    cylinder(d=vcc_wire_d, h=20, center=true);
    translate([-pside_sz_x/2, 0, 0]) {
      cube([pside_sz_x, vcc_wire_d, 20], center=true);
    }
  }
  translate([(pside_sz_x-switch_sz_x)/2-3, pside_sz_y/2, 10]) {
    cube([switch_sz_x, 40, switch_sz_z], center=true);
  }
}
translate([pside_sz_x/2, 0, 0]) {
  rotate([0, 0, 90]) {
    vent(x_size=pside_sz_y-10, z_size=pside_sz_z-10, y_side=wall, z_count=5, x_count=15, center=true);
  }
}

