// 16x02 lcd
lcd_l = 71;
lcd_w = 24;

interhole_l = 75.5;
interhole_w = 31.5;


module lcd_box_hole(h) {
  cube([lcd_l, lcd_w, h], center=true);
}

module lcd_holes(h, d) {
  for(xsgn=[-1,1]) {
    for(ysgn=[-1,1]) {
      translate([xsgn*interhole_l/2, ysgn*interhole_w/2, 0]) {
        cylinder(d=d, h=h, center=true);
      }
    }
  }
}

