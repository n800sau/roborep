$fn = 30;

wall = 2;

difference() {
  union() {
    cube([12, wall, 26]);
    cube([12, 10, wall]);
  }
  translate([5+0.9, 0, 19]) {
    rotate([90, 0, 0]) {
      cylinder(d=3.6, h=20, center=true);
    }
  }
}