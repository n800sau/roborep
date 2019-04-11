$fn = 10;
d = 2;
scale([1, 1, 0.5]) {
  difference() {
    union() {
      for(x=[-10:10]) {
        for(y=[-10:10]) {
          translate([x*d, y*d, 1]) {
            sphere(d=d, center=true);
          }
        }
      }
      translate([0, 0, 0]) {
        cube([42, 42, 2], center=true);
      }
    }
    translate([0, 0, -50.5]) {
      cube([100, 100, 100], center=true);
    }
  }
}
