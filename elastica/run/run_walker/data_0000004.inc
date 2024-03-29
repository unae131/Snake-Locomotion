#declare Ribbon_Spline0 =
spline { natural_spline
-0.25, <7.41412e-06,0.101694,2.79608>,
0, <6.24173e-06,-0.771684,2.84935>,
0.25, <5.06627e-06,-1.64506,2.90261>,
0.5, <3.88468e-06,-2.51839,2.95565>,
0.75, <2.80186e-06,-3.39176,3.00785>,
1, <1.87345e-06,-4.26511,3.05901>,
1.25, <1.30821e-06,-5.13858,3.10828>,
1.5, <9.81718e-07,-6.0121,3.15648>,
1.75, <7.75442e-07,-6.88565,3.20417>,
2, <6.01294e-07,-7.75922,3.25163>,
2.25, <3.63757e-07,-8.63281,3.29888>,
2.5, <-6.22692e-08,-9.50641,3.34569>,
2.75, <-8.12457e-07,-10.3801,3.39153>,
3, <-1.71108e-06,-11.2538,3.43676>,
3.25, <-2.70605e-06,-12.1276,3.48145>,
3.5, <-3.81791e-06,-13.0015,3.52525>,
3.75, <-4.93532e-06,-13.8754,3.56903>
}
union{
#local Nr = -0.25;
#local EndNr = 3.7;
#while (Nr <= EndNr)
box{ <-1.75,0,0>,
<5.25,-0.3,0.68>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate x*0
translate Ribbon_Spline0(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
#declare Ribbon_Spline2 =
spline { natural_spline
-0.25, <5.06603e-06,-1.64506,2.90261>,
0, <6.00376e-06,-1.67396,2.43686>,
0.25, <6.95421e-06,-1.70287,1.97112>,
0.5, <7.92278e-06,-1.73179,1.50539>,
0.75, <8.91463e-06,-1.76073,1.03967>,
1, <1.00109e-05,-1.78967,0.573955>,
1.25, <1.11214e-05,-1.81861,0.108253>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate -x*3.55396
translate Ribbon_Spline2(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
#declare Ribbon_Spline3 =
spline { natural_spline
-0.25, <-2.70616e-06,-12.1276,3.48145>,
0, <-2.21035e-06,-12.1556,2.9155>,
0.25, <-1.7073e-06,-12.1835,2.34956>,
0.5, <-1.18575e-06,-12.2115,1.78363>,
0.75, <-6.47634e-07,-12.2394,1.21772>,
1, <-3.95988e-08,-12.2673,0.651814>,
1.25, <5.83316e-07,-12.2952,0.0859236>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate -x*2.82533
translate Ribbon_Spline3(Nr)
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
#local Nr = Nr + 0.04;
#end
}
sphere_sweep
{
b_spline
25,
<-2.05,-14.35,1.5>, 0.29,
<-0.964286,-14.35,1.5>, 0.29,
<0.121429,-14.35,1.5>, 0.29,
<1.20714,-14.35,1.5>, 0.29,
<2.29286,-14.35,1.5>, 0.29,
<3.37857,-14.35,1.5>, 0.29,
<4.46429,-14.35,1.5>, 0.29,
<5.55,-14.35,1.5>, 0.29,
<5.55,-13.2,1.5>, 0.29,
<5.55,-12.05,1.5>, 0.29,
<5.55,-10.9,1.5>, 0.29,
<5.55,-9.75,1.5>, 0.29,
<4.46429,-9.75,1.5>, 0.29,
<3.37857,-9.75,1.5>, 0.29,
<2.29286,-9.75,1.5>, 0.29,
<1.20714,-9.75,1.5>, 0.29,
<0.121429,-9.75,1.5>, 0.29,
<-0.964286,-9.75,1.5>, 0.29,
<-2.05,-9.75,1.5>, 0.29,
<-2.05,-10.9,1.5>, 0.29,
<-2.05,-12.05,1.5>, 0.29,
<-2.05,-13.2,1.5>, 0.29,
<-2.05,-14.35,1.5>, 0.29,
<-0.964286,-14.35,1.5>, 0.29,
<0.121429,-14.35,1.5>, 0.29
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-2.05,-14.35,0.9>, 0.29,
<-0.964286,-14.35,0.9>, 0.29,
<0.121429,-14.35,0.9>, 0.29,
<1.20714,-14.35,0.9>, 0.29,
<2.29286,-14.35,0.9>, 0.29,
<3.37857,-14.35,0.9>, 0.29,
<4.46429,-14.35,0.9>, 0.29,
<5.55,-14.35,0.9>, 0.29,
<5.55,-13.2,0.9>, 0.29,
<5.55,-12.05,0.9>, 0.29,
<5.55,-10.9,0.9>, 0.29,
<5.55,-9.75,0.9>, 0.29,
<4.46429,-9.75,0.9>, 0.29,
<3.37857,-9.75,0.9>, 0.29,
<2.29286,-9.75,0.9>, 0.29,
<1.20714,-9.75,0.9>, 0.29,
<0.121429,-9.75,0.9>, 0.29,
<-0.964286,-9.75,0.9>, 0.29,
<-2.05,-9.75,0.9>, 0.29,
<-2.05,-10.9,0.9>, 0.29,
<-2.05,-12.05,0.9>, 0.29,
<-2.05,-13.2,0.9>, 0.29,
<-2.05,-14.35,0.9>, 0.29,
<-0.964286,-14.35,0.9>, 0.29,
<0.121429,-14.35,0.9>, 0.29
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-2.05,0.33,1.5>, 0.29,
<-0.964286,0.33,1.5>, 0.29,
<0.121429,0.33,1.5>, 0.29,
<1.20714,0.33,1.5>, 0.29,
<2.29286,0.33,1.5>, 0.29,
<3.37857,0.33,1.5>, 0.29,
<4.46429,0.33,1.5>, 0.29,
<5.55,0.33,1.5>, 0.29,
<5.55,-0.82,1.5>, 0.29,
<5.55,-1.97,1.5>, 0.29,
<5.55,-3.12,1.5>, 0.29,
<5.55,-4.27,1.5>, 0.29,
<4.46429,-4.27,1.5>, 0.29,
<3.37857,-4.27,1.5>, 0.29,
<2.29286,-4.27,1.5>, 0.29,
<1.20714,-4.27,1.5>, 0.29,
<0.121429,-4.27,1.5>, 0.29,
<-0.964286,-4.27,1.5>, 0.29,
<-2.05,-4.27,1.5>, 0.29,
<-2.05,-3.12,1.5>, 0.29,
<-2.05,-1.97,1.5>, 0.29,
<-2.05,-0.82,1.5>, 0.29,
<-2.05,0.33,1.5>, 0.29,
<-0.964286,0.33,1.5>, 0.29,
<0.121429,0.33,1.5>, 0.29
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
25,
<-2.05,0.33,0.9>, 0.29,
<-0.964286,0.33,0.9>, 0.29,
<0.121429,0.33,0.9>, 0.29,
<1.20714,0.33,0.9>, 0.29,
<2.29286,0.33,0.9>, 0.29,
<3.37857,0.33,0.9>, 0.29,
<4.46429,0.33,0.9>, 0.29,
<5.55,0.33,0.9>, 0.29,
<5.55,-0.82,0.9>, 0.29,
<5.55,-1.97,0.9>, 0.29,
<5.55,-3.12,0.9>, 0.29,
<5.55,-4.27,0.9>, 0.29,
<4.46429,-4.27,0.9>, 0.29,
<3.37857,-4.27,0.9>, 0.29,
<2.29286,-4.27,0.9>, 0.29,
<1.20714,-4.27,0.9>, 0.29,
<0.121429,-4.27,0.9>, 0.29,
<-0.964286,-4.27,0.9>, 0.29,
<-2.05,-4.27,0.9>, 0.29,
<-2.05,-3.12,0.9>, 0.29,
<-2.05,-1.97,0.9>, 0.29,
<-2.05,-0.82,0.9>, 0.29,
<-2.05,0.33,0.9>, 0.29,
<-0.964286,0.33,0.9>, 0.29,
<0.121429,0.33,0.9>, 0.29
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<3.37857,-4.27,1.5>, 0.227,
<3.37857,-4.72667,1.5>, 0.227,
<3.37857,-5.18333,1.5>, 0.227,
<3.37857,-5.64,1.5>, 0.227,
<3.37857,-6.09667,1.5>, 0.227,
<3.37857,-6.55333,1.5>, 0.227,
<3.37857,-7.01,1.5>, 0.227,
<3.37857,-7.46667,1.5>, 0.227,
<3.37857,-7.92333,1.5>, 0.227,
<3.37857,-8.38,1.5>, 0.227,
<3.37857,-8.83667,1.5>, 0.227,
<3.37857,-9.29333,1.5>, 0.227,
<3.37857,-9.75,1.5>, 0.227
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<2.29286,-4.27,1.5>, 0.227,
<2.29286,-4.72667,1.5>, 0.227,
<2.29286,-5.18333,1.5>, 0.227,
<2.29286,-5.64,1.5>, 0.227,
<2.29286,-6.09667,1.5>, 0.227,
<2.29286,-6.55333,1.5>, 0.227,
<2.29286,-7.01,1.5>, 0.227,
<2.29286,-7.46667,1.5>, 0.227,
<2.29286,-7.92333,1.5>, 0.227,
<2.29286,-8.38,1.5>, 0.227,
<2.29286,-8.83667,1.5>, 0.227,
<2.29286,-9.29333,1.5>, 0.227,
<2.29286,-9.75,1.5>, 0.227
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<1.20714,-4.27,1.5>, 0.227,
<1.20714,-4.72667,1.5>, 0.227,
<1.20714,-5.18333,1.5>, 0.227,
<1.20714,-5.64,1.5>, 0.227,
<1.20714,-6.09667,1.5>, 0.227,
<1.20714,-6.55333,1.5>, 0.227,
<1.20714,-7.01,1.5>, 0.227,
<1.20714,-7.46667,1.5>, 0.227,
<1.20714,-7.92333,1.5>, 0.227,
<1.20714,-8.38,1.5>, 0.227,
<1.20714,-8.83667,1.5>, 0.227,
<1.20714,-9.29333,1.5>, 0.227,
<1.20714,-9.75,1.5>, 0.227
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
sphere_sweep
{
b_spline
13,
<0.121429,-4.27,1.5>, 0.227,
<0.121429,-4.72667,1.5>, 0.227,
<0.121429,-5.18333,1.5>, 0.227,
<0.121429,-5.64,1.5>, 0.227,
<0.121429,-6.09667,1.5>, 0.227,
<0.121429,-6.55333,1.5>, 0.227,
<0.121429,-7.01,1.5>, 0.227,
<0.121429,-7.46667,1.5>, 0.227,
<0.121429,-7.92333,1.5>, 0.227,
<0.121429,-8.38,1.5>, 0.227,
<0.121429,-8.83667,1.5>, 0.227,
<0.121429,-9.29333,1.5>, 0.227,
<0.121429,-9.75,1.5>, 0.227
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}
