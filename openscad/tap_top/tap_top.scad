$fn = 50;

top_d = 50;
top_h = 2.5;
border_d = 43.3;
border_w = 5;
border_h = 5.6;
cut_count = 3;
cut_w = 10;

module tap_top(disk_only=false) {

  rotate([180, 0, 0]) {
    cylinder(d=top_d, h=disk_only ? 1 : top_h, center=true);
    if(!disk_only) {
      translate([0, 0, -(border_h+top_h)/2+1]) {
        difference() {
          cylinder(d=border_d, h=border_h+1, center=true);
          cylinder(d=border_d-border_w*2, h=border_h+1, center=true);
          translate([0, 0, -1]) {
            for(i=[0:cut_count]) {
              rotate([0, 0, 360/cut_count*i]) {
                cube([cut_w, top_d, border_h], center=true);
              }
            }
          }
        }
      }
    }
  }
}

tap_top(true);