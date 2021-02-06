package frc.paths;

import com.team319.trajectory.Path;

public class FullAutonp2 extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,19.6440,-11.2800,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,0.0040,0.2000,10.0000,0.0000,-0.0000},
				{0.0200,19.6520,-11.2800,0.0120,0.4000,10.0001,0.0065,0.0120,0.4000,10.0000,0.0000,0.0120,0.4000,9.9999,-0.0065,-0.0000},
				{0.0200,19.6640,-11.2800,0.0240,0.6000,9.9997,-0.0216,0.0240,0.6000,10.0000,0.0000,0.0240,0.6000,10.0003,0.0216,0.0000},
				{0.0200,19.6800,-11.2800,0.0400,0.8000,9.9977,-0.0974,0.0400,0.8000,10.0000,0.0000,0.0400,0.8000,10.0023,0.0974,0.0000},
				{0.0200,19.7000,-11.2800,0.0600,0.9998,9.9927,-0.2535,0.0600,1.0000,10.0000,0.0000,0.0600,1.0002,10.0073,0.2535,0.0000},
				{0.0200,19.7240,-11.2800,0.0840,1.1995,9.9824,-0.5119,0.0840,1.2000,10.0000,0.0000,0.0840,1.2005,10.0176,0.5119,0.0000},
				{0.0200,19.7520,-11.2800,0.1120,1.3987,9.9646,-0.8933,0.1120,1.4000,10.0000,0.0000,0.1120,1.4013,10.0354,0.8933,0.0000},
				{0.0200,19.7840,-11.2800,0.1439,1.5975,9.9363,-1.4155,0.1440,1.6000,10.0000,0.0000,0.1441,1.6025,10.0637,1.4155,0.0001},
				{0.0200,19.8200,-11.2800,0.1798,1.7954,9.8944,-2.0925,0.1800,1.8000,10.0000,0.0000,0.1802,1.8046,10.1056,2.0925,0.0002},
				{0.0200,19.8600,-11.2800,0.2197,1.9921,9.8357,-2.9341,0.2200,2.0000,10.0000,0.0000,0.2203,2.0079,10.1643,2.9341,0.0003},
				{0.0200,19.9040,-11.2800,0.2634,2.1872,9.7568,-3.9455,0.2640,2.2000,10.0000,0.0000,0.2646,2.2128,10.2432,3.9455,0.0006},
				{0.0200,19.9520,-11.2799,0.3110,2.3803,9.6543,-5.1266,0.3120,2.4000,10.0000,0.0000,0.3130,2.4197,10.3457,5.1266,0.0009},
				{0.0200,20.0040,-11.2799,0.3624,2.5708,9.5249,-6.4718,0.3640,2.6000,10.0000,0.0000,0.3656,2.6292,10.4751,6.4718,0.0015},
				{0.0200,20.0600,-11.2798,0.4176,2.7581,9.3655,-7.9699,0.4200,2.8000,10.0000,0.0000,0.4224,2.8419,10.6345,7.9698,0.0022},
				{0.0200,20.1200,-11.2796,0.4764,2.9416,9.1734,-9.6034,0.4800,3.0000,10.0000,0.0000,0.4836,3.0584,10.8266,9.6032,0.0033},
				{0.0200,20.1840,-11.2793,0.5388,3.1205,8.9464,-11.3486,0.5440,3.2000,10.0000,0.0000,0.5492,3.2795,11.0536,11.3483,0.0048},
				{0.0200,20.2520,-11.2790,0.6047,3.2942,8.6829,-13.1750,0.6120,3.4000,10.0000,0.0000,0.6193,3.5058,11.3171,13.1745,0.0067},
				{0.0200,20.3240,-11.2784,0.6739,3.4618,8.3820,-15.0446,0.6840,3.6000,10.0000,0.0000,0.6941,3.7382,11.6179,15.0437,0.0093},
				{0.0200,20.4000,-11.2776,0.7464,3.6227,8.0438,-16.9103,0.7600,3.8000,10.0000,0.0000,0.7736,3.9773,11.9561,16.9089,0.0125},
				{0.0200,20.4800,-11.2764,0.8219,3.7761,7.6695,-18.7145,0.8400,4.0000,10.0000,0.0000,0.8581,4.2239,12.3304,18.7124,0.0167},
				{0.0200,20.5640,-11.2748,0.9003,3.9213,7.2618,-20.3854,0.9240,4.2000,10.0000,0.0000,0.9477,4.4787,12.7380,20.3822,0.0218},
				{0.0200,20.6519,-11.2726,0.9815,4.0578,6.8252,-21.8326,1.0120,4.4000,10.0000,0.0000,1.0425,4.7422,13.1746,21.8281,0.0281},
				{0.0200,20.7439,-11.2697,1.0652,4.1851,6.3663,-22.9410,1.1040,4.6000,10.0000,0.0000,1.1428,5.0148,13.6333,22.9348,0.0358},
				{0.0200,20.8398,-11.2658,1.1513,4.3030,5.8951,-23.5629,1.2000,4.8000,10.0000,0.0000,1.2487,5.2969,14.1044,23.5546,0.0449},
				{0.0200,20.9397,-11.2608,1.2395,4.4115,5.4249,-23.5094,1.3000,5.0000,10.0000,0.0000,1.3605,5.5884,14.5743,23.4986,0.0558},
				{0.0200,21.0435,-11.2543,1.3297,4.5110,4.9741,-22.5414,1.4040,5.2000,10.0000,0.0000,1.4783,5.8889,15.0249,22.5279,0.0685},
				{0.0200,21.1512,-11.2462,1.4218,4.6023,4.5668,-20.3643,1.5120,5.4000,10.0000,0.0000,1.6022,6.1976,15.4318,20.3480,0.0832},
				{0.0200,21.2627,-11.2360,1.5155,4.6870,4.2342,-16.6290,1.6240,5.6000,10.0000,0.0000,1.7325,6.5128,15.7640,16.6102,0.1000},
				{0.0200,21.3780,-11.2233,1.6109,4.7673,4.0153,-10.9463,1.7400,5.8000,10.0000,0.0000,1.8691,6.8325,15.9826,10.9260,0.1190},
				{0.0200,21.4970,-11.2078,1.7078,4.8465,3.9569,-2.9215,1.8600,6.0000,10.0000,0.0000,2.0122,7.1533,16.0406,2.9016,0.1403},
				{0.0200,21.6117,-11.1903,1.8000,4.6129,-11.6784,-781.7623,1.9760,5.8000,-10.0000,0.0000,2.1519,6.9869,-8.3219,-1218.1238,0.1622},
				{0.0200,21.7220,-11.1710,1.8880,4.3986,-10.7148,48.1805,2.0880,5.6000,-10.0000,0.0000,2.2880,6.8012,-9.2851,-48.1607,0.1843},
				{0.0200,21.8279,-11.1501,1.9721,4.2029,-9.7863,46.4242,2.1960,5.4000,-10.0000,0.0000,2.4199,6.5969,-10.2132,-46.4063,0.2064},
				{0.0200,21.9295,-11.1277,2.0526,4.0244,-8.9246,43.0864,2.3000,5.2000,-10.0000,0.0000,2.5474,6.3754,-11.0746,-43.0717,0.2280},
				{0.0200,22.0267,-11.1040,2.1298,3.8613,-8.1552,38.4666,2.4000,5.0000,-10.0000,0.0000,2.6702,6.1385,-11.8438,-38.4558,0.2490},
				{0.0200,22.1195,-11.0794,2.2040,3.7114,-7.4966,32.9316,2.4960,4.8000,-10.0000,0.0000,2.7879,5.8885,-12.5023,-32.9251,0.2691},
				{0.0200,22.2079,-11.0541,2.2755,3.5722,-6.9592,26.8711,2.5880,4.6000,-10.0000,0.0000,2.9005,5.6277,-13.0396,-26.8687,0.2880},
				{0.0200,22.2921,-11.0284,2.3443,3.4413,-6.5461,20.6531,2.6760,4.4000,-10.0000,0.0000,3.0077,5.3586,-13.4527,-20.6541,0.3057},
				{0.0200,22.3720,-11.0025,2.4106,3.3162,-6.2543,14.5902,2.7600,4.2000,-10.0000,0.0000,3.1094,5.0837,-13.7446,-14.5938,0.3220},
				{0.0200,22.4477,-10.9766,2.4745,3.1946,-6.0759,8.9200,2.8400,4.0000,-10.0000,0.0000,3.2055,4.8053,-13.9231,-8.9255,0.3369},
				{0.0200,22.5192,-10.9510,2.5360,3.0746,-5.9999,3.8011,2.9160,3.8000,-10.0000,0.0000,3.2960,4.5253,-13.9993,-3.8076,0.3502},
				{0.0200,22.5867,-10.9259,2.5951,2.9544,-6.0135,-0.6803,2.9880,3.6000,-10.0000,0.0000,3.3809,4.2456,-13.9858,0.6735,0.3621},
				{0.0200,22.6502,-10.9014,2.6517,2.8323,-6.1034,-4.4960,3.0560,3.4000,-10.0000,0.0000,3.4602,3.9677,-13.8960,4.4894,0.3726},
				{0.0200,22.7097,-10.8779,2.7059,2.7072,-6.2566,-7.6593,3.1200,3.2000,-10.0000,0.0000,3.5341,3.6928,-13.7429,7.6532,0.3817},
				{0.0200,22.7652,-10.8553,2.7574,2.5780,-6.4608,-10.2089,3.1800,3.0000,-10.0000,0.0000,3.6025,3.4220,-13.5389,10.2036,0.3895},
				{0.0200,22.8170,-10.8339,2.8063,2.4439,-6.7047,-12.1960,3.2360,2.8000,-10.0000,0.0000,3.6656,3.1561,-13.2950,12.1915,0.3960},
				{0.0200,22.8649,-10.8137,2.8524,2.3043,-6.9782,-13.6742,3.2880,2.6000,-10.0000,0.0000,3.7236,2.8957,-13.0216,13.6706,0.4015},
				{0.0200,22.9090,-10.7948,2.8956,2.1589,-7.2721,-14.6934,3.3360,2.4000,-10.0000,0.0000,3.7764,2.6411,-12.7278,14.6906,0.4059},
				{0.0200,22.9494,-10.7774,2.9357,2.0073,-7.5780,-15.2967,3.3800,2.2000,-10.0000,0.0000,3.8242,2.3927,-12.4219,15.2946,0.4095},
				{0.0200,22.9861,-10.7614,2.9727,1.8495,-7.8884,-15.5194,3.4200,2.0000,-10.0000,0.0000,3.8672,2.1505,-12.1116,15.5179,0.4122},
				{0.0200,23.0191,-10.7469,3.0064,1.6856,-8.1962,-15.3901,3.4560,1.8000,-10.0000,0.0000,3.9055,1.9144,-11.8038,15.3890,0.4143},
				{0.0200,23.0484,-10.7340,3.0367,1.5157,-8.4948,-14.9319,3.4880,1.6000,-10.0000,0.0000,3.9392,1.6843,-11.5052,14.9312,0.4159},
				{0.0200,23.0740,-10.7227,3.0635,1.3402,-8.7781,-14.1654,3.5160,1.4000,-10.0000,0.0000,3.9684,1.4598,-11.2219,14.1650,0.4170},
				{0.0200,23.0959,-10.7130,3.0867,1.1593,-9.0403,-13.1108,3.5400,1.2000,-10.0000,0.0000,3.9932,1.2407,-10.9597,13.1106,0.4178},
				{0.0200,23.1142,-10.7048,3.1062,0.9738,-9.2761,-11.7898,3.5600,1.0000,-10.0000,0.0000,4.0138,1.0262,-10.7239,11.7897,0.4182},
				{0.0200,23.1288,-10.6983,3.1219,0.7842,-9.4807,-10.2275,3.5760,0.8000,-10.0000,0.0000,4.0301,0.8158,-10.5193,10.2275,0.4185},
				{0.0200,23.1398,-10.6935,3.1337,0.5912,-9.6498,-8.4537,3.5880,0.6000,-10.0000,0.0000,4.0422,0.6088,-10.3502,8.4537,0.4187},
				{0.0200,23.1471,-10.6902,3.1416,0.3956,-9.7798,-6.5027,3.5960,0.4000,-10.0000,0.0000,4.0503,0.4044,-10.2202,6.5027,0.4188},
				{0.0200,23.1507,-10.6886,3.1456,0.1983,-9.8681,-4.4140,3.6000,0.2000,-10.0000,0.0000,4.0544,0.2017,-10.1319,4.4140,0.4188},
				{0.0200,23.1507,-10.6886,3.1456,0.0000,-9.9127,-2.2310,3.6000,-0.0000,-10.0000,0.0000,4.0544,0.0000,-10.0873,2.2310,0.4188},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}