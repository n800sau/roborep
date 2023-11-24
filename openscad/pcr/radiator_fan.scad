wall = 2;
$fn = 50;

fan_air_hole_d = 42;
fan_x_hole_dst = 42;
fan_y_hole_dst = 31.5;
fan_hole_d = 3.7;

rad_sz_x = 17;
rad_sz_y = 43;
rad_sz_z = 43;
rad_gap = 1;
rad_hole_d = 4;

module fan_plate() {
  difference() {
    minkowski()
    {
      cube([fan_x_hole_dst, rad_sz_y-4, wall/2], center=true);
      cylinder(r=5, h=wall/2, center=true);
    }
    cylinder(d=fan_air_hole_d, h=wall, center=true);
    for(ypos=[-fan_y_hole_dst/2, fan_y_hole_dst/2]) {
      for(xpos=[-fan_x_hole_dst/2, fan_x_hole_dst/2]) {
        translate([xpos, ypos, 0]) {
          cylinder(d=fan_hole_d, h=wall, center=true);
        }
      }
    }
  }
}

module fan2rad() {
  fan_plate();
  translate([wall, 0, (-rad_sz_z+2*rad_gap)/2-wall]) {
    difference() {
      cube([rad_sz_x+2*rad_gap+2*wall, rad_sz_y+2*rad_gap+2*wall, rad_sz_z+2*rad_gap+wall], center=true);
      cube([rad_sz_x+2*rad_gap, rad_sz_y+2*rad_gap, rad_sz_z+2*rad_gap+wall], center=true);
      cube([rad_sz_x+2*rad_gap+wall*2, rad_sz_y-14, rad_sz_z+2*rad_gap+wall], center=true);
      translate([wall, 0, 0]) {
        cube([rad_sz_x+2*rad_gap+wall*2, rad_sz_y-6, rad_sz_z+2*rad_gap+wall], center=true);
      }
      for(zpos=[-14, 14]) {
        for(ypos=[-18, 18]) {
          translate([-10, ypos, zpos]) {
            rotate([0, 90, 0]) {
              cylinder(d=rad_hole_d, h=10, center=true);
            }
          }
        }
      }
    }
  }
  
}

module base2fan() {
  fan_plate();
}


//fan2rad();

base2fan();

