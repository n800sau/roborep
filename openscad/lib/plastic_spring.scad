// draw an arc width height h between radius r1..r2 and angles a1..a2
module arc(r1, r2, h, a1=0, a2=0) {
  if(a2-a1<=180) {
    difference() {
      cylinder(r=r2,h=h);
      translate([0,0,-1]) {
        cylinder(r=r1,h=h+2);
      }
      rotate(a2) {
        translate([-r1-r2,0,-1]) {
          cube([2*(r1+r2),2*(r1+r2),h+2]);
        }
      }
      rotate(a1+180) {
        translate([-r1-r2,0,-1]) {
          cube([2*(r1+r2),2*(r1+r2),h+2]);
        }
      }
    }
  } else {
    difference() {
      cylinder(r=r2,h=h);
      translate([0,0,-1]) {
        cylinder(r=r1,h=h+2);
      }
      intersection() {
	      rotate(a2) {
          translate([-r1-r2,0,-1]) {
            cube([2*(r1+r2),2*(r1+r2),h+2]);
          }
        }
	      rotate(a1+180) {
          translate([-r1-r2,0,-1]) {
            cube([2*(r1+r2),2*(r1+r2),h+2]);
          }
        }
	    }
    }
  }
}

                           
// sline - generate a "snake line" of width w and height h 
// with a arbitrary sequence of segments defined by a radius and a turning angle
//
//   angle[i] > 0  left turn / counter-clockwise
//   angle[i] < 0  left turn / clockwise
//   angle[i] = 0  straight segment with length radius[i]
//
                           
// Heinz Spiess, 2014-09-06 (CC BY-SA)
                           
module sline(angle,radius,i,w,h){
  r = abs(radius[i]);
  a = angle[i];
  scale([angle[i]>=0?1:-1,1,1])
  translate([a?r:0,0,0]) {
    translate([-w/2,-r-0.01,0]) cube([w,0.02,h]); // tiny overlap!
    if(a) {
      arc(r-w/2,r+w/2,0,a,h=h);
    } else if(r>0) {
      translate([-w/2,-r,0]) {
        cube([w,r,h]);
      }
    }
    if(i+1<len(angle)) {
      rotate(angle[i]) {
	      translate([a?-r:0,a?0:-r,0]) {
	       sline(angle,radius,i+1,w,h);
        }
      }
    }
  }
}

$fn = 50;

module plastic_spring(
  h = 20, // height
  d=20, // ?
  w = 2,  // case wall thickness
  ws = 2, // spring wall thickness
  el = 6, // ?
  l = 14, // len
  ){


  r = l + 2*ws;
  D = d + 2 * w - 2 * ws - 0.7;
      
  // plastic spring
  for(sy=[-1,1]) {
    scale([1,sy,1]) {
      translate([0,d/2+w-ws/2,0]) {
        rotate(-90) {
          sline([0,180,0,180,0,-180,0,90,0],
            [r, D/4, el, D/12, el/2, D/12, 1+el/2, D/5, D/3],0,ws, h);
        }
      }
    }
  }
}

plastic_spring(h=3);
translate([0, -15, 0]) {
  cube([2, 30, 6]);
}
