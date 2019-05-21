$fn = 36;

// 16x02 lcd
lcd_l = 71;
lcd_w = 24;

interhole_l = 75.5;
interhole_w = 31.5;

lcd_holes_shift_z = 2;

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

module lcd_all_holes(h=10, d=3.5) {
  translate([0, 0, lcd_holes_shift_z]) {
    lcd_holes(d=d, h=h);
  }
  lcd_box_hole(h=h);
}

lcd_all_holes();
