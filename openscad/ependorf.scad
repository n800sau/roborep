// Copyright 2018 Shusy
// This file is licensed Creative Commons Attribution-Non-Commercial 3.0.
// https://creativecommons.org/licenses/by-nc/3.0/

// You can get this file from https://www.thingiverse.com/thing:2816500

// modified by n800s

$fn=100*1;

tube_type = "0.5ml";

if(tube_type == "2ml") {
  main(38, 11.5, skirt_h=0);
} else if(tube_type == "0.5ml") {
  // 0.5ml
  main(30, 8, skirt_h=2);
} else {
  // big one
  main(75, 12, h_offset=30);
}

module main(
  tube_h,
  tube_d,
  h_offset = 10,
  //Number of tube along a axis
  na=2,
  //Number of tube along b axis
  nb=2,
  //wall thickness
  wall=2,
  skirt_h=2, // 0 - no skirt
) {

  //Holdr heigth
  hh=tube_h - h_offset;

  module base() {
    hull(){
      cylinder(d=(tube_d+wall),h=hh);
      translate([(tube_d+wall)*(na-1)+wall,0,0])
        cylinder(d=(tube_d+wall),h=hh); 
      translate([0,(tube_d+wall)*(nb-1)+wall,0])
        cylinder(d=(tube_d+wall),h=hh); 
      translate([(tube_d+wall)*(na-1)+wall,(tube_d+wall)*(nb-1)+wall,0])
        cylinder(d=(tube_d+wall),h=hh);
    }
    if(skirt_h > 0){
      hull(){
        translate([0,0,0])
          cylinder(d=tube_d*2,h=skirt_h);
        translate([(tube_d+wall)*(na-1)+wall,0,0])
          cylinder(d=tube_d*2,h=skirt_h); 
        translate([0,(tube_d+wall)*(nb-1)+wall,0])
          cylinder(d=tube_d*2,h=skirt_h); 
        translate([(tube_d+wall)*(na-1)+wall,(tube_d+wall)*(nb-1)+wall,0])
          cylinder(d=tube_d*2,h=skirt_h); 
        }
      }
  }
  
  module hole(){
     // cylinder(d2=11.6,d1=4,h=1,$fn=10); 
     // translate([0,0,1])
      cylinder(d=tube_d + 0.1,h=hh);
  }

  difference(){
      base();
      translate([0,0,1])
      for(i=[0:na-1]){
          for(j=[0:nb-1]){translate([i*(tube_d+wall)+wall/2,j*(tube_d+wall)+wall/2,0])hole();
          }
      }
  }
}

