#declare Ribbon_Spline0 =
spline { natural_spline
-0.25, <0.00014074,-0.117206,2.77062>,
0, <0.000266587,-0.980733,2.91186>,
0.25, <0.000392271,-1.84426,3.05309>,
0.5, <0.000517988,-2.70726,3.19412>,
0.75, <0.000643097,-3.571,3.33058>,
1, <0.000766921,-4.43524,3.45923>,
1.25, <0.000886436,-5.30274,3.56366>,
1.5, <0.00100193,-6.17286,3.64333>,
1.75, <0.00111311,-7.04493,3.69778>,
2, <0.00121881,-7.91822,3.72662>,
2.25, <0.00131791,-8.79198,3.72954>,
2.5, <0.00140994,-9.66543,3.70632>,
2.75, <0.0014958,-10.5378,3.65682>,
3, <0.00158009,-11.4103,3.59864>,
3.25, <0.00166344,-12.2825,3.53527>,
3.5, <0.00174528,-13.1548,3.4665>,
3.75, <0.00182724,-14.027,3.39772>
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
-0.25, <0.000392271,-1.84426,3.05309>,
0, <0.000372374,-1.92233,2.59302>,
0.25, <0.000352467,-2.00046,2.13295>,
0.5, <0.000332611,-2.07863,1.67291>,
0.75, <0.000312715,-2.15676,1.21287>,
1, <0.000292846,-2.23486,0.752842>,
1.25, <0.000273002,-2.31296,0.292823>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate -x*9.63699
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
-0.25, <0.00166344,-12.2825,3.53527>,
0, <0.00161249,-12.2347,2.97064>,
0.25, <0.00156141,-12.1868,2.40603>,
0.5, <0.00151025,-12.1388,1.84144>,
0.75, <0.00145891,-12.0909,1.27686>,
1, <0.00140749,-12.0431,0.712294>,
1.25, <0.00135598,-11.9952,0.147742>
}
union{
#local Nr = -0.25;
#local EndNr = 1.2;
#while (Nr <= EndNr)
box{ <-1.75,1.75,0>,
<5.25,-1.75,0.3>
pigment{ color rgbt<0.45,0.39,1,0>}
rotate x*4.84733
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
<-1.45504,-14.15,1.5>, 0.314002,
<-0.528961,-14.1501,1.5>, 0.314002,
<0.397109,-14.1501,1.5>, 0.314004,
<1.32317,-14.1501,1.5>, 0.314004,
<2.24924,-14.1501,1.5>, 0.314004,
<3.17531,-14.1501,1.5>, 0.314003,
<4.10138,-14.1501,1.5>, 0.314006,
<5.02741,-14.1501,1.5>, 0.311018,
<5.50967,-13.2521,1.5>, 0.309826,
<5.5464,-12.2568,1.5>, 0.311595,
<5.51675,-11.261,1.5>, 0.311572,
<5.51764,-10.2647,1.5>, 0.29813,
<4.45115,-9.90139,1.5>, 0.285183,
<3.39249,-9.53968,1.5>, 0.286769,
<2.2985,-9.40715,1.5>, 0.287985,
<1.19857,-9.40912,1.5>, 0.287973,
<0.105128,-9.5475,1.5>, 0.286679,
<-0.950884,-9.92054,1.5>, 0.285043,
<-2.01493,-10.294,1.5>, 0.297872,
<-2.01829,-11.2927,1.5>, 0.311181,
<-2.05119,-12.2909,1.5>, 0.311215,
<-2.00622,-13.2882,1.5>, 0.309365,
<-1.45504,-14.15,1.5>, 0.298735,
<-0.535145,-13.5733,1.5>, 0.289998,
<0.383658,-12.9949,1.5>, 0.289999
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
<-1.44525,-14.0957,0.9>, 0.31438,
<-0.521393,-14.0958,0.9>, 0.31438,
<0.402451,-14.0958,0.9>, 0.314382,
<1.32629,-14.0959,0.9>, 0.314382,
<2.25013,-14.0959,0.9>, 0.314382,
<3.17397,-14.096,0.9>, 0.31438,
<4.09783,-14.096,0.9>, 0.314384,
<5.02162,-14.096,0.9>, 0.311792,
<5.50517,-13.2074,0.9>, 0.310988,
<5.54391,-12.2196,0.9>, 0.312745,
<5.51102,-11.2311,0.9>, 0.312718,
<5.50839,-10.2422,0.9>, 0.299716,
<4.45115,-9.90139,0.9>, 0.286196,
<3.39249,-9.53968,0.9>, 0.286769,
<2.2985,-9.40715,0.9>, 0.287985,
<1.19857,-9.40912,0.9>, 0.287973,
<0.105128,-9.5475,0.9>, 0.286679,
<-0.950884,-9.92054,0.9>, 0.286026,
<-2.0058,-10.273,0.9>, 0.299399,
<-2.01319,-11.2648,0.9>, 0.312268,
<-2.04973,-12.256,0.9>, 0.312307,
<-2.00188,-13.2461,0.9>, 0.310463,
<-1.44525,-14.0957,0.9>, 0.299284,
<-0.531459,-13.5094,0.9>, 0.289998,
<0.381215,-12.9213,0.9>, 0.289999
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
<-1.47099,-0.0462843,1.5>, 0.31337,
<-0.541172,-0.0460754,1.5>, 0.31337,
<0.388637,-0.0458759,1.5>, 0.313371,
<1.31844,-0.0456812,1.5>, 0.313372,
<2.24825,-0.0454873,1.5>, 0.313372,
<3.17805,-0.04529,1.5>, 0.31337,
<4.10787,-0.0450921,1.5>, 0.313374,
<5.03763,-0.0448958,1.5>, 0.310244,
<5.53441,-0.94196,1.5>, 0.309076,
<5.5674,-1.94109,1.5>, 0.311008,
<5.53375,-2.94062,1.5>, 0.310984,
<5.53034,-3.9406,1.5>, 0.297322,
<4.45734,-4.31018,1.5>, 0.284514,
<3.39556,-4.67024,1.5>, 0.286459,
<2.29909,-4.80205,1.5>, 0.287674,
<1.19678,-4.80021,1.5>, 0.287662,
<0.100799,-4.66281,1.5>, 0.286367,
<-0.958498,-4.2917,1.5>, 0.284378,
<-2.02926,-3.91297,1.5>, 0.297077,
<-2.03713,-2.91059,1.5>, 0.310607,
<-2.0743,-1.90876,1.5>, 0.310642,
<-2.03404,-0.907599,1.5>, 0.308629,
<-1.47099,-0.0462844,1.5>, 0.298285,
<-0.556688,-0.631841,1.5>, 0.289998,
<0.356553,-1.21902,1.5>, 0.289999
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
<-1.4491,-0.141251,0.9>, 0.314243,
<-0.524439,-0.141017,0.9>, 0.314243,
<0.40021,-0.140794,0.9>, 0.314245,
<1.32485,-0.140573,0.9>, 0.314245,
<2.2495,-0.140348,0.9>, 0.314245,
<3.17415,-0.140124,0.9>, 0.314243,
<4.09881,-0.139899,0.9>, 0.314248,
<5.02341,-0.139676,0.9>, 0.311819,
<5.51721,-1.0212,0.9>, 0.311265,
<5.55665,-2.0067,0.9>, 0.313106,
<5.52087,-2.99282,0.9>, 0.313076,
<5.51457,-3.97947,0.9>, 0.300093,
<4.45734,-4.31018,0.9>, 0.28624,
<3.39556,-4.67024,0.9>, 0.286459,
<2.29909,-4.80205,0.9>, 0.287674,
<1.19678,-4.80021,0.9>, 0.287662,
<0.100799,-4.66281,0.9>, 0.286367,
<-0.958498,-4.2917,0.9>, 0.28606,
<-2.01351,-3.94896,0.9>, 0.299752,
<-2.02516,-2.95932,0.9>, 0.312593,
<-2.06517,-1.9703,0.9>, 0.312636,
<-2.01685,-0.9823,0.9>, 0.310705,
<-1.4491,-0.14125,0.9>, 0.29936,
<-0.543173,-0.739686,0.9>, 0.289998,
<0.361647,-1.33976,0.9>, 0.289999
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
<3.39556,-4.67024,1.5>, 0.240811,
<3.39532,-5.07603,1.5>, 0.240811,
<3.39507,-5.48182,1.5>, 0.240811,
<3.39483,-5.8876,1.5>, 0.240811,
<3.39458,-6.29339,1.5>, 0.240812,
<3.39433,-6.69917,1.5>, 0.240812,
<3.39408,-7.10496,1.5>, 0.240812,
<3.39382,-7.51074,1.5>, 0.240811,
<3.39356,-7.91653,1.5>, 0.240811,
<3.3933,-8.32231,1.5>, 0.240811,
<3.39303,-8.7281,1.5>, 0.240811,
<3.39276,-9.13389,1.5>, 0.240811,
<3.39249,-9.53968,1.5>, 0.240811
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
<2.29909,-4.80206,1.5>, 0.247626,
<2.29905,-5.18582,1.5>, 0.247626,
<2.29901,-5.56958,1.5>, 0.247627,
<2.29897,-5.95334,1.5>, 0.247626,
<2.29893,-6.33709,1.5>, 0.247627,
<2.29888,-6.72085,1.5>, 0.247627,
<2.29884,-7.1046,1.5>, 0.247627,
<2.29878,-7.48836,1.5>, 0.247627,
<2.29873,-7.87211,1.5>, 0.247626,
<2.29868,-8.25587,1.5>, 0.247626,
<2.29862,-8.63963,1.5>, 0.247626,
<2.29856,-9.02339,1.5>, 0.247627,
<2.2985,-9.40715,1.5>, 0.247626
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
<1.19678,-4.80021,1.5>, 0.247523,
<1.19692,-5.18429,1.5>, 0.247524,
<1.19706,-5.56837,1.5>, 0.247524,
<1.1972,-5.95245,1.5>, 0.247524,
<1.19735,-6.33652,1.5>, 0.247524,
<1.1975,-6.72059,1.5>, 0.247525,
<1.19764,-7.10467,1.5>, 0.247525,
<1.19779,-7.48874,1.5>, 0.247524,
<1.19795,-7.87281,1.5>, 0.247524,
<1.1981,-8.25689,1.5>, 0.247524,
<1.19826,-8.64097,1.5>, 0.247524,
<1.19841,-9.02504,1.5>, 0.247524,
<1.19857,-9.40912,1.5>, 0.247523
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
<0.100799,-4.66282,1.5>, 0.240435,
<0.101149,-5.06987,1.5>, 0.240435,
<0.101498,-5.47693,1.5>, 0.240435,
<0.101847,-5.88399,1.5>, 0.240435,
<0.102204,-6.29105,1.5>, 0.240435,
<0.102561,-6.6981,1.5>, 0.240435,
<0.102917,-7.10516,1.5>, 0.240435,
<0.103282,-7.51221,1.5>, 0.240435,
<0.103647,-7.91927,1.5>, 0.240435,
<0.104012,-8.32633,1.5>, 0.240435,
<0.104384,-8.73338,1.5>, 0.240435,
<0.104756,-9.14044,1.5>, 0.240435,
<0.105128,-9.5475,1.5>, 0.240435
texture
{
pigment{ color rgb<1,0.65,0> }
finish { phong 1 }
}
scale<0.25,0.25,0.25>
rotate<0,90,90>
translate<2,0,4>
}