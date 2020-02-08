package frc.paths;

import com.team319.trajectory.Path;

public class eckert extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,41.6772,5.5400,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,3.1416},
				{0.0200,41.6716,5.5400,0.0084,0.2800,7.0000,0.0000,0.0084,0.2800,7.0000,0.0000,0.0084,0.2800,7.0000,0.0000,3.1416},
				{0.0200,41.6632,5.5400,0.0168,0.4200,7.0000,-0.0000,0.0168,0.4200,7.0000,0.0000,0.0168,0.4200,7.0000,-0.0000,3.1416},
				{0.0200,41.6520,5.5400,0.0280,0.5600,7.0000,-0.0000,0.0280,0.5600,7.0000,0.0000,0.0280,0.5600,7.0000,-0.0000,3.1416},
				{0.0200,41.6380,5.5400,0.0420,0.7000,7.0000,0.0000,0.0420,0.7000,7.0000,0.0000,0.0420,0.7000,7.0000,0.0000,3.1416},
				{0.0200,41.6212,5.5400,0.0588,0.8400,7.0000,-0.0000,0.0588,0.8400,7.0000,0.0000,0.0588,0.8400,7.0000,-0.0000,3.1416},
				{0.0200,41.6016,5.5400,0.0784,0.9800,7.0000,0.0000,0.0784,0.9800,7.0000,0.0000,0.0784,0.9800,7.0000,0.0000,3.1416},
				{0.0200,41.5792,5.5400,0.1008,1.1200,7.0000,-0.0000,0.1008,1.1200,7.0000,0.0000,0.1008,1.1200,7.0000,-0.0000,3.1416},
				{0.0200,41.5540,5.5400,0.1260,1.2600,7.0000,0.0000,0.1260,1.2600,7.0000,0.0000,0.1260,1.2600,7.0000,0.0000,3.1416},
				{0.0200,41.5260,5.5400,0.1540,1.4000,7.0000,-0.0000,0.1540,1.4000,7.0000,0.0000,0.1540,1.4000,7.0000,-0.0000,3.1416},
				{0.0200,41.4952,5.5400,0.1848,1.5400,7.0000,0.0000,0.1848,1.5400,7.0000,0.0000,0.1848,1.5400,7.0000,0.0000,3.1416},
				{0.0200,41.4616,5.5400,0.2184,1.6800,7.0000,0.0000,0.2184,1.6800,7.0000,0.0000,0.2184,1.6800,7.0000,0.0000,3.1416},
				{0.0200,41.4252,5.5400,0.2548,1.8200,7.0000,-0.0000,0.2548,1.8200,7.0000,0.0000,0.2548,1.8200,7.0000,-0.0000,3.1416},
				{0.0200,41.3860,5.5400,0.2940,1.9600,7.0000,-0.0000,0.2940,1.9600,7.0000,0.0000,0.2940,1.9600,7.0000,-0.0000,3.1416},
				{0.0200,41.3440,5.5400,0.3360,2.1000,7.0000,0.0000,0.3360,2.1000,7.0000,0.0000,0.3360,2.1000,7.0000,0.0000,3.1416},
				{0.0200,41.2992,5.5400,0.3808,2.2400,7.0000,0.0000,0.3808,2.2400,7.0000,0.0000,0.3808,2.2400,7.0000,0.0000,3.1416},
				{0.0200,41.2516,5.5400,0.4284,2.3800,7.0000,-0.0000,0.4284,2.3800,7.0000,0.0000,0.4284,2.3800,7.0000,-0.0000,3.1416},
				{0.0200,41.2012,5.5400,0.4788,2.5200,7.0000,0.0000,0.4788,2.5200,7.0000,0.0000,0.4788,2.5200,7.0000,0.0000,3.1416},
				{0.0200,41.1480,5.5400,0.5320,2.6600,7.0000,0.0000,0.5320,2.6600,7.0000,0.0000,0.5320,2.6600,7.0000,0.0000,3.1416},
				{0.0200,41.0920,5.5400,0.5880,2.8000,7.0000,0.0000,0.5880,2.8000,7.0000,0.0000,0.5880,2.8000,7.0000,0.0000,3.1416},
				{0.0200,41.0332,5.5400,0.6468,2.9400,7.0000,-0.0000,0.6468,2.9400,7.0000,0.0000,0.6468,2.9400,7.0000,-0.0000,3.1416},
				{0.0200,40.9716,5.5400,0.7084,3.0800,7.0000,0.0000,0.7084,3.0800,7.0000,0.0000,0.7084,3.0800,7.0000,0.0000,3.1416},
				{0.0200,40.9072,5.5400,0.7728,3.2200,7.0000,-0.0000,0.7728,3.2200,7.0000,0.0000,0.7728,3.2200,7.0000,-0.0000,3.1416},
				{0.0200,40.8400,5.5400,0.8400,3.3600,7.0000,-0.0000,0.8400,3.3600,7.0000,0.0000,0.8400,3.3600,7.0000,-0.0000,3.1416},
				{0.0200,40.7700,5.5400,0.9100,3.5000,7.0000,0.0000,0.9100,3.5000,7.0000,0.0000,0.9100,3.5000,7.0000,0.0000,3.1416},
				{0.0200,40.6972,5.5400,0.9828,3.6400,7.0000,-0.0000,0.9828,3.6400,7.0000,0.0000,0.9828,3.6400,7.0000,-0.0000,3.1416},
				{0.0200,40.6216,5.5400,1.0584,3.7800,7.0000,-0.0000,1.0584,3.7800,7.0000,0.0000,1.0584,3.7800,7.0000,-0.0000,3.1416},
				{0.0200,40.5432,5.5400,1.1368,3.9200,7.0000,0.0000,1.1368,3.9200,7.0000,0.0000,1.1368,3.9200,7.0000,0.0000,3.1416},
				{0.0200,40.4620,5.5400,1.2180,4.0600,7.0000,0.0000,1.2180,4.0600,7.0000,0.0000,1.2180,4.0600,7.0000,0.0000,3.1416},
				{0.0200,40.3780,5.5400,1.3020,4.2000,7.0000,0.0000,1.3020,4.2000,7.0000,0.0000,1.3020,4.2000,7.0000,0.0000,3.1416},
				{0.0200,40.2912,5.5400,1.3888,4.3400,7.0000,-0.0000,1.3888,4.3400,7.0000,0.0000,1.3888,4.3400,7.0000,-0.0000,3.1416},
				{0.0200,40.2016,5.5400,1.4784,4.4800,7.0000,0.0000,1.4784,4.4800,7.0000,0.0000,1.4784,4.4800,7.0000,0.0000,3.1416},
				{0.0200,40.1092,5.5400,1.5708,4.6200,7.0000,-0.0000,1.5708,4.6200,7.0000,0.0000,1.5708,4.6200,7.0000,-0.0000,3.1416},
				{0.0200,40.0140,5.5400,1.6660,4.7600,7.0000,0.0000,1.6660,4.7600,7.0000,0.0000,1.6660,4.7600,7.0000,0.0000,3.1416},
				{0.0200,39.9160,5.5400,1.7640,4.9000,7.0000,-0.0000,1.7640,4.9000,7.0000,0.0000,1.7640,4.9000,7.0000,-0.0000,3.1416},
				{0.0200,39.8152,5.5400,1.8648,5.0400,7.0000,0.0000,1.8648,5.0400,7.0000,0.0000,1.8648,5.0400,7.0000,0.0000,3.1416},
				{0.0200,39.7116,5.5400,1.9684,5.1800,7.0000,0.0000,1.9684,5.1800,7.0000,0.0000,1.9684,5.1800,7.0000,0.0000,3.1416},
				{0.0200,39.6052,5.5400,2.0748,5.3200,7.0000,-0.0000,2.0748,5.3200,7.0000,0.0000,2.0748,5.3200,7.0000,-0.0000,3.1416},
				{0.0200,39.4960,5.5400,2.1840,5.4600,7.0000,0.0000,2.1840,5.4600,7.0000,0.0000,2.1840,5.4600,7.0000,0.0000,3.1416},
				{0.0200,39.3840,5.5400,2.2960,5.6000,7.0000,0.0000,2.2960,5.6000,7.0000,0.0000,2.2960,5.6000,7.0000,0.0000,3.1416},
				{0.0200,39.2692,5.5400,2.4108,5.7400,7.0000,-0.0000,2.4108,5.7400,7.0000,0.0000,2.4108,5.7400,7.0000,-0.0000,3.1416},
				{0.0200,39.1516,5.5400,2.5284,5.8800,7.0000,0.0000,2.5284,5.8800,7.0000,0.0000,2.5284,5.8800,7.0000,0.0000,3.1416},
				{0.0200,39.0312,5.5400,2.6488,6.0200,7.0000,0.0000,2.6488,6.0200,7.0000,0.0000,2.6488,6.0200,7.0000,0.0000,3.1416},
				{0.0200,38.9080,5.5400,2.7720,6.1600,7.0000,-0.0000,2.7720,6.1600,7.0000,0.0000,2.7720,6.1600,7.0000,-0.0000,3.1416},
				{0.0200,38.7820,5.5400,2.8980,6.3000,7.0000,0.0000,2.8980,6.3000,7.0000,0.0000,2.8980,6.3000,7.0000,0.0000,3.1416},
				{0.0200,38.6532,5.5400,3.0268,6.4400,7.0000,-0.0000,3.0268,6.4400,7.0000,0.0000,3.0268,6.4400,7.0000,-0.0000,3.1416},
				{0.0200,38.5216,5.5400,3.1584,6.5800,7.0000,0.0000,3.1584,6.5800,7.0000,0.0000,3.1584,6.5800,7.0000,0.0000,3.1416},
				{0.0200,38.3872,5.5400,3.2928,6.7200,7.0000,0.0000,3.2928,6.7200,7.0000,0.0000,3.2928,6.7200,7.0000,0.0000,3.1416},
				{0.0200,38.2500,5.5400,3.4300,6.8600,7.0000,-0.0000,3.4300,6.8600,7.0000,0.0000,3.4300,6.8600,7.0000,-0.0000,3.1416},
				{0.0200,38.1100,5.5400,3.5700,7.0000,7.0000,0.0000,3.5700,7.0000,7.0000,0.0000,3.5700,7.0000,7.0000,0.0000,3.1416},
				{0.0200,37.9672,5.5400,3.7128,7.1400,7.0000,0.0000,3.7128,7.1400,7.0000,0.0000,3.7128,7.1400,7.0000,0.0000,3.1416},
				{0.0200,37.8216,5.5400,3.8584,7.2800,7.0000,-0.0000,3.8584,7.2800,7.0000,0.0000,3.8584,7.2800,7.0000,-0.0000,3.1416},
				{0.0200,37.6732,5.5400,4.0068,7.4200,7.0000,0.0000,4.0068,7.4200,7.0000,0.0000,4.0068,7.4200,7.0000,0.0000,3.1416},
				{0.0200,37.5220,5.5400,4.1580,7.5600,7.0000,0.0000,4.1580,7.5600,7.0000,0.0000,4.1580,7.5600,7.0000,0.0000,3.1416},
				{0.0200,37.3736,5.5400,4.3064,7.4200,-7.0000,-700.0000,4.3064,7.4200,-7.0000,0.0000,4.3064,7.4200,-7.0000,-700.0000,3.1416},
				{0.0200,37.2280,5.5400,4.4520,7.2800,-7.0000,-0.0000,4.4520,7.2800,-7.0000,0.0000,4.4520,7.2800,-7.0000,-0.0000,3.1416},
				{0.0200,37.0852,5.5400,4.5948,7.1400,-7.0000,0.0000,4.5948,7.1400,-7.0000,0.0000,4.5948,7.1400,-7.0000,0.0000,3.1416},
				{0.0200,36.9452,5.5400,4.7348,7.0000,-7.0000,-0.0000,4.7348,7.0000,-7.0000,0.0000,4.7348,7.0000,-7.0000,-0.0000,3.1416},
				{0.0200,36.8080,5.5400,4.8720,6.8600,-7.0000,0.0000,4.8720,6.8600,-7.0000,0.0000,4.8720,6.8600,-7.0000,0.0000,3.1416},
				{0.0200,36.6736,5.5400,5.0064,6.7200,-7.0000,-0.0000,5.0064,6.7200,-7.0000,0.0000,5.0064,6.7200,-7.0000,-0.0000,3.1416},
				{0.0200,36.5420,5.5400,5.1380,6.5800,-7.0000,0.0000,5.1380,6.5800,-7.0000,0.0000,5.1380,6.5800,-7.0000,0.0000,3.1416},
				{0.0200,36.4132,5.5400,5.2668,6.4400,-7.0000,-0.0000,5.2668,6.4400,-7.0000,0.0000,5.2668,6.4400,-7.0000,-0.0000,3.1416},
				{0.0200,36.2872,5.5400,5.3928,6.3000,-7.0000,0.0000,5.3928,6.3000,-7.0000,0.0000,5.3928,6.3000,-7.0000,0.0000,3.1416},
				{0.0200,36.1640,5.5400,5.5160,6.1600,-7.0000,0.0000,5.5160,6.1600,-7.0000,0.0000,5.5160,6.1600,-7.0000,0.0000,3.1416},
				{0.0200,36.0436,5.5400,5.6364,6.0200,-7.0000,0.0000,5.6364,6.0200,-7.0000,0.0000,5.6364,6.0200,-7.0000,0.0000,3.1416},
				{0.0200,35.9260,5.5400,5.7540,5.8800,-7.0000,0.0000,5.7540,5.8800,-7.0000,0.0000,5.7540,5.8800,-7.0000,0.0000,3.1416},
				{0.0200,35.8112,5.5400,5.8688,5.7400,-7.0000,-0.0000,5.8688,5.7400,-7.0000,0.0000,5.8688,5.7400,-7.0000,-0.0000,3.1416},
				{0.0200,35.6992,5.5400,5.9808,5.6000,-7.0000,0.0000,5.9808,5.6000,-7.0000,0.0000,5.9808,5.6000,-7.0000,0.0000,3.1416},
				{0.0200,35.5900,5.5400,6.0900,5.4600,-7.0000,-0.0000,6.0900,5.4600,-7.0000,0.0000,6.0900,5.4600,-7.0000,-0.0000,3.1416},
				{0.0200,35.4836,5.5400,6.1964,5.3200,-7.0000,0.0000,6.1964,5.3200,-7.0000,0.0000,6.1964,5.3200,-7.0000,0.0000,3.1416},
				{0.0200,35.3800,5.5400,6.3000,5.1800,-7.0000,0.0000,6.3000,5.1800,-7.0000,0.0000,6.3000,5.1800,-7.0000,0.0000,3.1416},
				{0.0200,35.2792,5.5400,6.4008,5.0400,-7.0000,0.0000,6.4008,5.0400,-7.0000,0.0000,6.4008,5.0400,-7.0000,0.0000,3.1416},
				{0.0200,35.1812,5.5400,6.4988,4.9000,-7.0000,-0.0000,6.4988,4.9000,-7.0000,0.0000,6.4988,4.9000,-7.0000,-0.0000,3.1416},
				{0.0200,35.0860,5.5400,6.5940,4.7600,-7.0000,0.0000,6.5940,4.7600,-7.0000,0.0000,6.5940,4.7600,-7.0000,0.0000,3.1416},
				{0.0200,34.9936,5.5400,6.6864,4.6200,-7.0000,0.0000,6.6864,4.6200,-7.0000,0.0000,6.6864,4.6200,-7.0000,0.0000,3.1416},
				{0.0200,34.9040,5.5400,6.7760,4.4800,-7.0000,0.0000,6.7760,4.4800,-7.0000,0.0000,6.7760,4.4800,-7.0000,0.0000,3.1416},
				{0.0200,34.8172,5.5400,6.8628,4.3400,-7.0000,-0.0000,6.8628,4.3400,-7.0000,0.0000,6.8628,4.3400,-7.0000,-0.0000,3.1416},
				{0.0200,34.7332,5.5400,6.9468,4.2000,-7.0000,0.0000,6.9468,4.2000,-7.0000,0.0000,6.9468,4.2000,-7.0000,0.0000,3.1416},
				{0.0200,34.6520,5.5400,7.0280,4.0600,-7.0000,0.0000,7.0280,4.0600,-7.0000,0.0000,7.0280,4.0600,-7.0000,0.0000,3.1416},
				{0.0200,34.5736,5.5400,7.1064,3.9200,-7.0000,-0.0000,7.1064,3.9200,-7.0000,0.0000,7.1064,3.9200,-7.0000,-0.0000,3.1416},
				{0.0200,34.4980,5.5400,7.1820,3.7800,-7.0000,0.0000,7.1820,3.7800,-7.0000,0.0000,7.1820,3.7800,-7.0000,0.0000,3.1416},
				{0.0200,34.4252,5.5400,7.2548,3.6400,-7.0000,0.0000,7.2548,3.6400,-7.0000,0.0000,7.2548,3.6400,-7.0000,0.0000,3.1416},
				{0.0200,34.3552,5.5400,7.3248,3.5000,-7.0000,-0.0000,7.3248,3.5000,-7.0000,0.0000,7.3248,3.5000,-7.0000,-0.0000,3.1416},
				{0.0200,34.2880,5.5400,7.3920,3.3600,-7.0000,0.0000,7.3920,3.3600,-7.0000,0.0000,7.3920,3.3600,-7.0000,0.0000,3.1416},
				{0.0200,34.2236,5.5400,7.4564,3.2200,-7.0000,0.0000,7.4564,3.2200,-7.0000,0.0000,7.4564,3.2200,-7.0000,0.0000,3.1416},
				{0.0200,34.1620,5.5400,7.5180,3.0800,-7.0000,0.0000,7.5180,3.0800,-7.0000,0.0000,7.5180,3.0800,-7.0000,0.0000,3.1416},
				{0.0200,34.1032,5.5400,7.5768,2.9400,-7.0000,-0.0000,7.5768,2.9400,-7.0000,0.0000,7.5768,2.9400,-7.0000,-0.0000,3.1416},
				{0.0200,34.0472,5.5400,7.6328,2.8000,-7.0000,0.0000,7.6328,2.8000,-7.0000,0.0000,7.6328,2.8000,-7.0000,0.0000,3.1416},
				{0.0200,33.9940,5.5400,7.6860,2.6600,-7.0000,0.0000,7.6860,2.6600,-7.0000,0.0000,7.6860,2.6600,-7.0000,0.0000,3.1416},
				{0.0200,33.9436,5.5400,7.7364,2.5200,-7.0000,0.0000,7.7364,2.5200,-7.0000,0.0000,7.7364,2.5200,-7.0000,0.0000,3.1416},
				{0.0200,33.8960,5.5400,7.7840,2.3800,-7.0000,-0.0000,7.7840,2.3800,-7.0000,0.0000,7.7840,2.3800,-7.0000,-0.0000,3.1416},
				{0.0200,33.8512,5.5400,7.8288,2.2400,-7.0000,0.0000,7.8288,2.2400,-7.0000,0.0000,7.8288,2.2400,-7.0000,0.0000,3.1416},
				{0.0200,33.8092,5.5400,7.8708,2.1000,-7.0000,-0.0000,7.8708,2.1000,-7.0000,0.0000,7.8708,2.1000,-7.0000,-0.0000,3.1416},
				{0.0200,33.7700,5.5400,7.9100,1.9600,-7.0000,0.0000,7.9100,1.9600,-7.0000,0.0000,7.9100,1.9600,-7.0000,0.0000,3.1416},
				{0.0200,33.7336,5.5400,7.9464,1.8200,-7.0000,0.0000,7.9464,1.8200,-7.0000,0.0000,7.9464,1.8200,-7.0000,0.0000,3.1416},
				{0.0200,33.7000,5.5400,7.9800,1.6800,-7.0000,0.0000,7.9800,1.6800,-7.0000,0.0000,7.9800,1.6800,-7.0000,0.0000,3.1416},
				{0.0200,33.6692,5.5400,8.0108,1.5400,-7.0000,0.0000,8.0108,1.5400,-7.0000,0.0000,8.0108,1.5400,-7.0000,0.0000,3.1416},
				{0.0200,33.6412,5.5400,8.0388,1.4000,-7.0000,-0.0000,8.0388,1.4000,-7.0000,0.0000,8.0388,1.4000,-7.0000,-0.0000,3.1416},
				{0.0200,33.6160,5.5400,8.0640,1.2600,-7.0000,0.0000,8.0640,1.2600,-7.0000,0.0000,8.0640,1.2600,-7.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.3000,-48.0000,-2050.0000,8.0700,1.1200,-7.0000,0.0000,8.0700,0.3000,-48.0000,-2050.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,-15.0000,1650.0000,8.0700,0.9800,-7.0000,0.0000,8.0700,0.0000,-15.0000,1650.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,750.0000,8.0700,0.8400,-7.0000,0.0000,8.0700,0.0000,0.0000,750.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,0.7000,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,0.5600,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,0.4200,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,0.2800,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,0.1400,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},
				{0.0200,33.6100,5.5400,8.0700,0.0000,0.0000,0.0000,8.0700,-0.0000,-7.0000,0.0000,8.0700,0.0000,0.0000,0.0000,3.1416},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}