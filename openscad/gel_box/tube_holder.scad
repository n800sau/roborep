use <MCAD/boxes.scad>

// 3 centrifuge tubes
// 3 tubes with blue top

d_cf = 11;
d_big = 12;

h = 35;
wall = 2;
pad = 5;

x_size = pad+(d_big+pad)*3;
y_size = d_big+d_cf+pad*3;

difference() {
  roundedBox([x_size, y_size, h], 2, true);
  translate([0, 0, -wall]) {
    roundedBox([x_size-wall*2, y_size-wall*2, h], 2, true);
  }
  translate([0, 0, -wall]) {
    roundedBox([x_size-20, y_size+20, h], 2, true);
  }
  translate([0, 0, -wall]) {
    roundedBox([x_size+20, y_size-20, h], 2, true);
  }
  translate([0, -(d_big+d_cf-pad)/2, 0]) {
    for(x=[-1:1]) {
      translate([(d_big+pad)*x, 0, 0]) {
        cylinder(d=d_cf, h=h*2, center=true);
        translate([0, pad+(d_cf+d_big)/2, 0]) {
          cylinder(d=d_big, h=h*2, center=true);
        }
      }
    }
  }
}
