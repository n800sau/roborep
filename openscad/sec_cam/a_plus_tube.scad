stlfile = "Raspberry_Pi_A+_holder.stl";

module rpi_a_plus() {
  difference() {
    translate([50, -65, 0]) {
      import(stlfile, convexity=10);
    }
    translate([-42.1, 0, 0]) {
      cube([10, 100, 50], center=true);
    }
    translate([32, 0, 0]) {
      cube([10, 100, 50], center=true);
    }
    translate([-5, -8, -20]) {
      cube([50, 45, 50], center=true);
    }
  }
}

module rpi_a_plus_tube(ext_fwd=0, ext_back=0) {
  cube_width = 64;
  cube_len = 69;
  cube_height = 50;
  cube_wall = 2;

  difference() {
    union() {
      difference() {
        translate([-5, -7.5+ext_fwd/2-ext_back/2, 0]) {
          cube([cube_width,
            cube_len+ext_fwd+ext_back,
            cube_height], center=true);
        }
        hull() {
          rpi_a_plus();
        }
      }
      rpi_a_plus();
    }
    translate([-5, -7.5+ext_fwd/2-ext_back/2, 0]) {
      cube([cube_width - cube_wall*2,
        cube_len+ext_fwd+ext_back,
        cube_height - cube_wall*2], center=true);
    }
  }
//  rpi_a_plus();
}

rpi_a_plus_tube();