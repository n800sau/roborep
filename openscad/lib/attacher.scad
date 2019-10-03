$fn = 40;

module attacher(length, width=10, wall=2, hole_d=3.6) {
  difference() {
    union() {
      cube([length, width, wall], center=true);
      for(i=[-1,1]) {
        translate([i*length/2, 0, 0]) {
          cylinder(d=width, h=wall, center=true);
        }
      }
    }
    for(i=[-1,1]) {
      translate([i*length/2, 0, 0]) {
        cylinder(d=hole_d, h=wall, center=true);
      }
    }
  }
}
