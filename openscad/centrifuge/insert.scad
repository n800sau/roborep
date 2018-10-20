$fn = 100;
d = 85;
sector_h = 4;

module pie_slice(r=3.0,a=30) {
  $fn=64;
  intersection() {
    circle(r=r);
    square(r);
    rotate(a-90) square(r);
  }
}


difference() {
  union() {
    linear_extrude(height = sector_h) {
      pie_slice(r=d/2, a=20);
    }
    rotate([0, 0, 10])
      translate([d/2-11, 0, sector_h])
//        cylinder(d=6.1, h=5);
        cube([6, 6, 5], center=true);
  }
    linear_extrude(height = sector_h) {
      pie_slice(r=26, a=30);
    }
}