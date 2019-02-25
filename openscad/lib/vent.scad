module vent(x_size=30, x_count=9, z_size=30, z_count=9, y_size=2, center=true) {
  x_off = center ? 0 : x_size/2;
  y_off = center ? 0 : y_size/2;
  z_off = center ? 0 : z_size/2;
  translate([x_off, y_off, z_off]) {
    x_step = x_size/(x_count+x_count+1);
    z_step = z_size/(z_count+z_count+1);
    for(xpos=[-x_size/2+x_step*1.5:x_step*2:x_size/2]) {
      translate([xpos, 0, 0]) {
        cube([x_step, y_size, z_size], center=true);
      }
    }
    for(zpos=[-z_size/2+z_step*1.5:z_step*2:z_size/2]) {
      translate([0, 0, zpos]) {
        cube([x_size, y_size, z_step], center=true);
      }
    }
    // test x_size
    translate([0, 0, -z_size/2-1]) {
      %cube([x_size, 1, 1], center=true);
    }
    // test z_size
    translate([x_size/2+1, 0, 0]) {
      %cube([1, 1, z_size], center=true);
    }
  }
}
