$fn = 50;
pole_d = 8.5;
pole_h = 70;

cone_h = 15;
cone_d = 12;

wall = 2;
plate_sz_x = 20;
plate_sz_y = 16;

hole_d_through = 3.2;

translate([0, 0, pole_h/2]) {
  cylinder(d=pole_d, h=pole_h, center=true);
  translate([0, 0, (-pole_h+cone_h)/2]) {
    cylinder(d1=cone_d, d2=pole_d, h=cone_h, center=true);
  }
  translate([0, 0, pole_h/2]) {
    sphere(d=pole_d, center=true);
  }
}
translate([0, 0, wall/2]) {
  difference() {
    cube([plate_sz_x, plate_sz_y, wall], center=true);
    for(x=[-1,1]) {
      for(y=[-1,1]) {
        translate([x*(plate_sz_x-wall-hole_d_through)/2, y*(plate_sz_y-wall-hole_d_through)/2, 0]) {
          cylinder(d=hole_d_through, h=wall, center=true);
        }
      }
    }
  }
}
