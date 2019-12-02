$fn = 30;
wall = 2;
d_sz_x = 4.5;
d_sz_y = 3*2.5;
d_sz_z = 2.5;
gap_sz_z = 1;

ex_sz_x = 12;
ex_sz_y = d_sz_y + 2 * wall;


translate([(-d_sz_x-ex_sz_x)/2, 0, -(d_sz_z+wall)/2]) {
  cube([ex_sz_x, ex_sz_y, wall], center=true);
  translate([-(ex_sz_x-5)/2, 0, 0]) {
    difference() {
      cube([ex_sz_x-5, ex_sz_y+8, wall], center=true);
      for(ysgn=[-1,1]) {
        translate([0, ysgn*(ex_sz_y+2)/2, 0]) {
          cylinder(d=2.5, h=10, center=true);
        }
      }
    }
  }
}
difference() {
  cube([d_sz_x, d_sz_y+2*wall, d_sz_z+2*wall], center=true);
  cube([d_sz_x, d_sz_y, d_sz_z], center=true);
  translate([0, 0, d_sz_z/2+wall/2]) {
    cube([d_sz_x, d_sz_y, wall], center=true);
  }
}

