w = 150;
h = 100;
th = 0.9;
pad = 4;
step_x = w/10;
step_y = h/10;
bar_w = 2;

difference() {
  cube([w+pad*2, h+pad*2, th]);
  translate([pad, pad, 0]) {
    cube([w, h, th]);
  }
}
for(x=[0:step_x:w]) {
//  echo("X=", x, w, step);
  translate([pad+x, pad, 0]) {
    cube([bar_w, h, th]);
  }
}
for(y=[0:step_y:h]) {
//  echo("Y=", y, h, step);
  translate([pad, pad+y, 0]) {
    cube([w, bar_w, th]);
  }
}
