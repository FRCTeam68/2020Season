package frc.paths;

import com.team319.trajectory.Path;

public class ratliff extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,41.4574,4.9400,0.0026,0.1300,6.5000,0.0000,0.0026,0.1300,6.5000,0.0000,0.0026,0.1300,6.5000,0.0000,3.1416},
				{0.0200,41.4522,4.9400,0.0078,0.2600,6.5000,0.0000,0.0078,0.2600,6.5000,0.0000,0.0078,0.2600,6.5000,0.0000,3.1416},
				{0.0200,41.4444,4.9400,0.0156,0.3900,6.5000,-0.0000,0.0156,0.3900,6.5000,0.0000,0.0156,0.3900,6.5000,-0.0000,3.1416},
				{0.0200,41.4340,4.9400,0.0260,0.5200,6.5000,0.0000,0.0260,0.5200,6.5000,0.0000,0.0260,0.5200,6.5000,0.0000,3.1416},
				{0.0200,41.4210,4.9400,0.0390,0.6500,6.5000,-0.0000,0.0390,0.6500,6.5000,0.0000,0.0390,0.6500,6.5000,-0.0000,3.1416},
				{0.0200,41.4054,4.9400,0.0546,0.7800,6.5000,0.0000,0.0546,0.7800,6.5000,0.0000,0.0546,0.7800,6.5000,0.0000,3.1416},
				{0.0200,41.3872,4.9400,0.0728,0.9100,6.5000,0.0000,0.0728,0.9100,6.5000,0.0000,0.0728,0.9100,6.5000,0.0000,3.1416},
				{0.0200,41.3664,4.9400,0.0936,1.0400,6.5000,0.0000,0.0936,1.0400,6.5000,0.0000,0.0936,1.0400,6.5000,0.0000,3.1416},
				{0.0200,41.3430,4.9400,0.1170,1.1700,6.5000,0.0000,0.1170,1.1700,6.5000,0.0000,0.1170,1.1700,6.5000,0.0000,3.1416},
				{0.0200,41.3170,4.9400,0.1430,1.3000,6.5000,-0.0000,0.1430,1.3000,6.5000,0.0000,0.1430,1.3000,6.5000,-0.0000,3.1416},
				{0.0200,41.2884,4.9400,0.1716,1.4300,6.5000,0.0000,0.1716,1.4300,6.5000,0.0000,0.1716,1.4300,6.5000,0.0000,3.1416},
				{0.0200,41.2572,4.9400,0.2028,1.5600,6.5000,-0.0000,0.2028,1.5600,6.5000,0.0000,0.2028,1.5600,6.5000,-0.0000,3.1416},
				{0.0200,41.2234,4.9400,0.2366,1.6900,6.5000,0.0000,0.2366,1.6900,6.5000,0.0000,0.2366,1.6900,6.5000,0.0000,3.1416},
				{0.0200,41.1870,4.9400,0.2730,1.8200,6.5000,-0.0000,0.2730,1.8200,6.5000,0.0000,0.2730,1.8200,6.5000,-0.0000,3.1416},
				{0.0200,41.1480,4.9400,0.3120,1.9500,6.5000,0.0000,0.3120,1.9500,6.5000,0.0000,0.3120,1.9500,6.5000,0.0000,3.1416},
				{0.0200,41.1064,4.9400,0.3536,2.0800,6.5000,-0.0000,0.3536,2.0800,6.5000,0.0000,0.3536,2.0800,6.5000,-0.0000,3.1416},
				{0.0200,41.0622,4.9400,0.3978,2.2100,6.5000,-0.0000,0.3978,2.2100,6.5000,0.0000,0.3978,2.2100,6.5000,-0.0000,3.1416},
				{0.0200,41.0154,4.9400,0.4446,2.3400,6.5000,0.0000,0.4446,2.3400,6.5000,0.0000,0.4446,2.3400,6.5000,0.0000,3.1416},
				{0.0200,40.9660,4.9400,0.4940,2.4700,6.5000,-0.0000,0.4940,2.4700,6.5000,0.0000,0.4940,2.4700,6.5000,-0.0000,3.1416},
				{0.0200,40.9140,4.9400,0.5460,2.6000,6.5000,0.0000,0.5460,2.6000,6.5000,0.0000,0.5460,2.6000,6.5000,0.0000,3.1416},
				{0.0200,40.8594,4.9400,0.6006,2.7300,6.5000,-0.0000,0.6006,2.7300,6.5000,0.0000,0.6006,2.7300,6.5000,-0.0000,3.1416},
				{0.0200,40.8022,4.9400,0.6578,2.8600,6.5000,0.0000,0.6578,2.8600,6.5000,0.0000,0.6578,2.8600,6.5000,0.0000,3.1416},
				{0.0200,40.7424,4.9400,0.7176,2.9900,6.5000,0.0000,0.7176,2.9900,6.5000,0.0000,0.7176,2.9900,6.5000,0.0000,3.1416},
				{0.0200,40.6800,4.9400,0.7800,3.1200,6.5000,-0.0000,0.7800,3.1200,6.5000,0.0000,0.7800,3.1200,6.5000,-0.0000,3.1416},
				{0.0200,40.6150,4.9400,0.8450,3.2500,6.5000,-0.0000,0.8450,3.2500,6.5000,0.0000,0.8450,3.2500,6.5000,-0.0000,3.1416},
				{0.0200,40.5474,4.9400,0.9126,3.3800,6.5000,0.0000,0.9126,3.3800,6.5000,0.0000,0.9126,3.3800,6.5000,0.0000,3.1416},
				{0.0200,40.4772,4.9400,0.9828,3.5100,6.5000,0.0000,0.9828,3.5100,6.5000,0.0000,0.9828,3.5100,6.5000,0.0000,3.1416},
				{0.0200,40.4044,4.9400,1.0556,3.6400,6.5000,0.0000,1.0556,3.6400,6.5000,0.0000,1.0556,3.6400,6.5000,0.0000,3.1416},
				{0.0200,40.3290,4.9400,1.1310,3.7700,6.5000,0.0000,1.1310,3.7700,6.5000,0.0000,1.1310,3.7700,6.5000,0.0000,3.1416},
				{0.0200,40.2510,4.9400,1.2090,3.9000,6.5000,-0.0000,1.2090,3.9000,6.5000,0.0000,1.2090,3.9000,6.5000,-0.0000,3.1416},
				{0.0200,40.1704,4.9400,1.2896,4.0300,6.5000,0.0000,1.2896,4.0300,6.5000,0.0000,1.2896,4.0300,6.5000,0.0000,3.1416},
				{0.0200,40.0872,4.9400,1.3728,4.1600,6.5000,-0.0000,1.3728,4.1600,6.5000,0.0000,1.3728,4.1600,6.5000,-0.0000,3.1416},
				{0.0200,40.0014,4.9400,1.4586,4.2900,6.5000,-0.0000,1.4586,4.2900,6.5000,0.0000,1.4586,4.2900,6.5000,-0.0000,3.1416},
				{0.0200,39.9130,4.9400,1.5470,4.4200,6.5000,0.0000,1.5470,4.4200,6.5000,0.0000,1.5470,4.4200,6.5000,0.0000,3.1416},
				{0.0200,39.8220,4.9400,1.6380,4.5500,6.5000,-0.0000,1.6380,4.5500,6.5000,0.0000,1.6380,4.5500,6.5000,-0.0000,3.1416},
				{0.0200,39.7284,4.9400,1.7316,4.6800,6.5000,0.0000,1.7316,4.6800,6.5000,0.0000,1.7316,4.6800,6.5000,0.0000,3.1416},
				{0.0200,39.6322,4.9400,1.8278,4.8100,6.5000,0.0000,1.8278,4.8100,6.5000,0.0000,1.8278,4.8100,6.5000,0.0000,3.1416},
				{0.0200,39.5334,4.9400,1.9266,4.9400,6.5000,0.0000,1.9266,4.9400,6.5000,0.0000,1.9266,4.9400,6.5000,0.0000,3.1416},
				{0.0200,39.4320,4.9400,2.0280,5.0700,6.5000,-0.0000,2.0280,5.0700,6.5000,0.0000,2.0280,5.0700,6.5000,-0.0000,3.1416},
				{0.0200,39.3280,4.9400,2.1320,5.2000,6.5000,0.0000,2.1320,5.2000,6.5000,0.0000,2.1320,5.2000,6.5000,0.0000,3.1416},
				{0.0200,39.2214,4.9400,2.2386,5.3300,6.5000,0.0000,2.2386,5.3300,6.5000,0.0000,2.2386,5.3300,6.5000,0.0000,3.1416},
				{0.0200,39.1122,4.9400,2.3478,5.4600,6.5000,0.0000,2.3478,5.4600,6.5000,0.0000,2.3478,5.4600,6.5000,0.0000,3.1416},
				{0.0200,39.0004,4.9400,2.4596,5.5900,6.5000,-0.0000,2.4596,5.5900,6.5000,0.0000,2.4596,5.5900,6.5000,-0.0000,3.1416},
				{0.0200,38.8860,4.9400,2.5740,5.7200,6.5000,0.0000,2.5740,5.7200,6.5000,0.0000,2.5740,5.7200,6.5000,0.0000,3.1416},
				{0.0200,38.7690,4.9400,2.6910,5.8500,6.5000,0.0000,2.6910,5.8500,6.5000,0.0000,2.6910,5.8500,6.5000,0.0000,3.1416},
				{0.0200,38.6494,4.9400,2.8106,5.9800,6.5000,-0.0000,2.8106,5.9800,6.5000,0.0000,2.8106,5.9800,6.5000,-0.0000,3.1416},
				{0.0200,38.5272,4.9400,2.9328,6.1100,6.5000,0.0000,2.9328,6.1100,6.5000,0.0000,2.9328,6.1100,6.5000,0.0000,3.1416},
				{0.0200,38.4024,4.9400,3.0576,6.2400,6.5000,0.0000,3.0576,6.2400,6.5000,0.0000,3.0576,6.2400,6.5000,0.0000,3.1416},
				{0.0200,38.2750,4.9400,3.1850,6.3700,6.5000,-0.0000,3.1850,6.3700,6.5000,0.0000,3.1850,6.3700,6.5000,-0.0000,3.1416},
				{0.0200,38.1450,4.9400,3.3150,6.5000,6.5000,0.0000,3.3150,6.5000,6.5000,0.0000,3.3150,6.5000,6.5000,0.0000,3.1416},
				{0.0200,38.0124,4.9400,3.4476,6.6300,6.5000,-0.0000,3.4476,6.6300,6.5000,0.0000,3.4476,6.6300,6.5000,-0.0000,3.1416},
				{0.0200,37.8772,4.9400,3.5828,6.7600,6.5000,0.0000,3.5828,6.7600,6.5000,0.0000,3.5828,6.7600,6.5000,0.0000,3.1416},
				{0.0200,37.7394,4.9400,3.7206,6.8900,6.5000,0.0000,3.7206,6.8900,6.5000,0.0000,3.7206,6.8900,6.5000,0.0000,3.1416},
				{0.0200,37.5990,4.9400,3.8610,7.0200,6.5000,-0.0000,3.8610,7.0200,6.5000,0.0000,3.8610,7.0200,6.5000,-0.0000,3.1416},
				{0.0200,37.4560,4.9400,4.0040,7.1500,6.5000,0.0000,4.0040,7.1500,6.5000,0.0000,4.0040,7.1500,6.5000,0.0000,3.1416},
				{0.0200,37.3104,4.9400,4.1496,7.2800,6.5000,-0.0000,4.1496,7.2800,6.5000,0.0000,4.1496,7.2800,6.5000,-0.0000,3.1416},
				{0.0200,37.1622,4.9400,4.2978,7.4100,6.5000,0.0000,4.2978,7.4100,6.5000,0.0000,4.2978,7.4100,6.5000,0.0000,3.1416},
				{0.0200,37.0114,4.9400,4.4486,7.5400,6.5000,-0.0000,4.4486,7.5400,6.5000,0.0000,4.4486,7.5400,6.5000,-0.0000,3.1416},
				{0.0200,36.8580,4.9400,4.6020,7.6700,6.5000,0.0000,4.6020,7.6700,6.5000,0.0000,4.6020,7.6700,6.5000,0.0000,3.1416},
				{0.0200,36.7020,4.9400,4.7580,7.8000,6.5000,-0.0000,4.7580,7.8000,6.5000,0.0000,4.7580,7.8000,6.5000,-0.0000,3.1416},
				{0.0200,36.5434,4.9400,4.9166,7.9300,6.5000,0.0000,4.9166,7.9300,6.5000,0.0000,4.9166,7.9300,6.5000,0.0000,3.1416},
				{0.0200,36.3822,4.9400,5.0778,8.0600,6.5000,-0.0000,5.0778,8.0600,6.5000,0.0000,5.0778,8.0600,6.5000,-0.0000,3.1416},
				{0.0200,36.2184,4.9400,5.2416,8.1900,6.5000,0.0000,5.2416,8.1900,6.5000,0.0000,5.2416,8.1900,6.5000,0.0000,3.1416},
				{0.0200,36.0520,4.9400,5.4080,8.3200,6.5000,0.0000,5.4080,8.3200,6.5000,0.0000,5.4080,8.3200,6.5000,0.0000,3.1416},
				{0.0200,35.8882,4.9400,5.5718,8.1900,-6.5000,-650.0000,5.5718,8.1900,-6.5000,0.0000,5.5718,8.1900,-6.5000,-650.0000,3.1416},
				{0.0200,35.7270,4.9400,5.7330,8.0600,-6.5000,-0.0000,5.7330,8.0600,-6.5000,0.0000,5.7330,8.0600,-6.5000,-0.0000,3.1416},
				{0.0200,35.5684,4.9400,5.8916,7.9300,-6.5000,0.0000,5.8916,7.9300,-6.5000,0.0000,5.8916,7.9300,-6.5000,0.0000,3.1416},
				{0.0200,35.4124,4.9400,6.0476,7.8000,-6.5000,0.0000,6.0476,7.8000,-6.5000,0.0000,6.0476,7.8000,-6.5000,0.0000,3.1416},
				{0.0200,35.2590,4.9400,6.2010,7.6700,-6.5000,0.0000,6.2010,7.6700,-6.5000,0.0000,6.2010,7.6700,-6.5000,0.0000,3.1416},
				{0.0200,35.1082,4.9400,6.3518,7.5400,-6.5000,-0.0000,6.3518,7.5400,-6.5000,0.0000,6.3518,7.5400,-6.5000,-0.0000,3.1416},
				{0.0200,34.9600,4.9400,6.5000,7.4100,-6.5000,0.0000,6.5000,7.4100,-6.5000,0.0000,6.5000,7.4100,-6.5000,0.0000,3.1416},
				{0.0200,34.8144,4.9400,6.6456,7.2800,-6.5000,0.0000,6.6456,7.2800,-6.5000,0.0000,6.6456,7.2800,-6.5000,0.0000,3.1416},
				{0.0200,34.6714,4.9400,6.7886,7.1500,-6.5000,0.0000,6.7886,7.1500,-6.5000,0.0000,6.7886,7.1500,-6.5000,0.0000,3.1416},
				{0.0200,34.5310,4.9400,6.9290,7.0200,-6.5000,-0.0000,6.9290,7.0200,-6.5000,0.0000,6.9290,7.0200,-6.5000,-0.0000,3.1416},
				{0.0200,34.3932,4.9400,7.0668,6.8900,-6.5000,0.0000,7.0668,6.8900,-6.5000,0.0000,7.0668,6.8900,-6.5000,0.0000,3.1416},
				{0.0200,34.2580,4.9400,7.2020,6.7600,-6.5000,0.0000,7.2020,6.7600,-6.5000,0.0000,7.2020,6.7600,-6.5000,0.0000,3.1416},
				{0.0200,34.1254,4.9400,7.3346,6.6300,-6.5000,-0.0000,7.3346,6.6300,-6.5000,0.0000,7.3346,6.6300,-6.5000,-0.0000,3.1416},
				{0.0200,33.9954,4.9400,7.4646,6.5000,-6.5000,0.0000,7.4646,6.5000,-6.5000,0.0000,7.4646,6.5000,-6.5000,0.0000,3.1416},
				{0.0200,33.8680,4.9400,7.5920,6.3700,-6.5000,-0.0000,7.5920,6.3700,-6.5000,0.0000,7.5920,6.3700,-6.5000,-0.0000,3.1416},
				{0.0200,33.7432,4.9400,7.7168,6.2400,-6.5000,0.0000,7.7168,6.2400,-6.5000,0.0000,7.7168,6.2400,-6.5000,0.0000,3.1416},
				{0.0200,33.6210,4.9400,7.8390,6.1100,-6.5000,-0.0000,7.8390,6.1100,-6.5000,0.0000,7.8390,6.1100,-6.5000,-0.0000,3.1416},
				{0.0200,33.5014,4.9400,7.9586,5.9800,-6.5000,0.0000,7.9586,5.9800,-6.5000,0.0000,7.9586,5.9800,-6.5000,0.0000,3.1416},
				{0.0200,33.3844,4.9400,8.0756,5.8500,-6.5000,-0.0000,8.0756,5.8500,-6.5000,0.0000,8.0756,5.8500,-6.5000,-0.0000,3.1416},
				{0.0200,33.2700,4.9400,8.1900,5.7200,-6.5000,0.0000,8.1900,5.7200,-6.5000,0.0000,8.1900,5.7200,-6.5000,0.0000,3.1416},
				{0.0200,33.1582,4.9400,8.3018,5.5900,-6.5000,-0.0000,8.3018,5.5900,-6.5000,0.0000,8.3018,5.5900,-6.5000,-0.0000,3.1416},
				{0.0200,33.0490,4.9400,8.4110,5.4600,-6.5000,0.0000,8.4110,5.4600,-6.5000,0.0000,8.4110,5.4600,-6.5000,0.0000,3.1416},
				{0.0200,32.9424,4.9400,8.5176,5.3300,-6.5000,-0.0000,8.5176,5.3300,-6.5000,0.0000,8.5176,5.3300,-6.5000,-0.0000,3.1416},
				{0.0200,32.8384,4.9400,8.6216,5.2000,-6.5000,0.0000,8.6216,5.2000,-6.5000,0.0000,8.6216,5.2000,-6.5000,0.0000,3.1416},
				{0.0200,32.7370,4.9400,8.7230,5.0700,-6.5000,-0.0000,8.7230,5.0700,-6.5000,0.0000,8.7230,5.0700,-6.5000,-0.0000,3.1416},
				{0.0200,32.6382,4.9400,8.8218,4.9400,-6.5000,0.0000,8.8218,4.9400,-6.5000,0.0000,8.8218,4.9400,-6.5000,0.0000,3.1416},
				{0.0200,32.5420,4.9400,8.9180,4.8100,-6.5000,-0.0000,8.9180,4.8100,-6.5000,0.0000,8.9180,4.8100,-6.5000,-0.0000,3.1416},
				{0.0200,32.4484,4.9400,9.0116,4.6800,-6.5000,0.0000,9.0116,4.6800,-6.5000,0.0000,9.0116,4.6800,-6.5000,0.0000,3.1416},
				{0.0200,32.3574,4.9400,9.1026,4.5500,-6.5000,-0.0000,9.1026,4.5500,-6.5000,0.0000,9.1026,4.5500,-6.5000,-0.0000,3.1416},
				{0.0200,32.2690,4.9400,9.1910,4.4200,-6.5000,0.0000,9.1910,4.4200,-6.5000,0.0000,9.1910,4.4200,-6.5000,0.0000,3.1416},
				{0.0200,32.1832,4.9400,9.2768,4.2900,-6.5000,0.0000,9.2768,4.2900,-6.5000,0.0000,9.2768,4.2900,-6.5000,0.0000,3.1416},
				{0.0200,32.1000,4.9400,9.3600,4.1600,-6.5000,0.0000,9.3600,4.1600,-6.5000,0.0000,9.3600,4.1600,-6.5000,0.0000,3.1416},
				{0.0200,32.0194,4.9400,9.4406,4.0300,-6.5000,-0.0000,9.4406,4.0300,-6.5000,0.0000,9.4406,4.0300,-6.5000,-0.0000,3.1416},
				{0.0200,31.9414,4.9400,9.5186,3.9000,-6.5000,0.0000,9.5186,3.9000,-6.5000,0.0000,9.5186,3.9000,-6.5000,0.0000,3.1416},
				{0.0200,31.8660,4.9400,9.5940,3.7700,-6.5000,-0.0000,9.5940,3.7700,-6.5000,0.0000,9.5940,3.7700,-6.5000,-0.0000,3.1416},
				{0.0200,31.7932,4.9400,9.6668,3.6400,-6.5000,0.0000,9.6668,3.6400,-6.5000,0.0000,9.6668,3.6400,-6.5000,0.0000,3.1416},
				{0.0200,31.7230,4.9400,9.7370,3.5100,-6.5000,0.0000,9.7370,3.5100,-6.5000,0.0000,9.7370,3.5100,-6.5000,0.0000,3.1416},
				{0.0200,31.6554,4.9400,9.8046,3.3800,-6.5000,-0.0000,9.8046,3.3800,-6.5000,0.0000,9.8046,3.3800,-6.5000,-0.0000,3.1416},
				{0.0200,31.5904,4.9400,9.8696,3.2500,-6.5000,0.0000,9.8696,3.2500,-6.5000,0.0000,9.8696,3.2500,-6.5000,0.0000,3.1416},
				{0.0200,31.5280,4.9400,9.9320,3.1200,-6.5000,0.0000,9.9320,3.1200,-6.5000,0.0000,9.9320,3.1200,-6.5000,0.0000,3.1416},
				{0.0200,31.4682,4.9400,9.9918,2.9900,-6.5000,0.0000,9.9918,2.9900,-6.5000,0.0000,9.9918,2.9900,-6.5000,0.0000,3.1416},
				{0.0200,31.4110,4.9400,10.0490,2.8600,-6.5000,-0.0000,10.0490,2.8600,-6.5000,0.0000,10.0490,2.8600,-6.5000,-0.0000,3.1416},
				{0.0200,31.3564,4.9400,10.1036,2.7300,-6.5000,0.0000,10.1036,2.7300,-6.5000,0.0000,10.1036,2.7300,-6.5000,0.0000,3.1416},
				{0.0200,31.3044,4.9400,10.1556,2.6000,-6.5000,-0.0000,10.1556,2.6000,-6.5000,0.0000,10.1556,2.6000,-6.5000,-0.0000,3.1416},
				{0.0200,31.2550,4.9400,10.2050,2.4700,-6.5000,-0.0000,10.2050,2.4700,-6.5000,0.0000,10.2050,2.4700,-6.5000,-0.0000,3.1416},
				{0.0200,31.2082,4.9400,10.2518,2.3400,-6.5000,0.0000,10.2518,2.3400,-6.5000,0.0000,10.2518,2.3400,-6.5000,0.0000,3.1416},
				{0.0200,31.1640,4.9400,10.2960,2.2100,-6.5000,-0.0000,10.2960,2.2100,-6.5000,0.0000,10.2960,2.2100,-6.5000,-0.0000,3.1416},
				{0.0200,31.1224,4.9400,10.3376,2.0800,-6.5000,0.0000,10.3376,2.0800,-6.5000,0.0000,10.3376,2.0800,-6.5000,0.0000,3.1416},
				{0.0200,31.0834,4.9400,10.3766,1.9500,-6.5000,-0.0000,10.3766,1.9500,-6.5000,0.0000,10.3766,1.9500,-6.5000,-0.0000,3.1416},
				{0.0200,31.0470,4.9400,10.4130,1.8200,-6.5000,0.0000,10.4130,1.8200,-6.5000,0.0000,10.4130,1.8200,-6.5000,0.0000,3.1416},
				{0.0200,31.0132,4.9400,10.4468,1.6900,-6.5000,-0.0000,10.4468,1.6900,-6.5000,0.0000,10.4468,1.6900,-6.5000,-0.0000,3.1416},
				{0.0200,30.9820,4.9400,10.4780,1.5600,-6.5000,0.0000,10.4780,1.5600,-6.5000,0.0000,10.4780,1.5600,-6.5000,0.0000,3.1416},
				{0.0200,30.9534,4.9400,10.5066,1.4300,-6.5000,-0.0000,10.5066,1.4300,-6.5000,0.0000,10.5066,1.4300,-6.5000,-0.0000,3.1416},
				{0.0200,30.9274,4.9400,10.5326,1.3000,-6.5000,0.0000,10.5326,1.3000,-6.5000,0.0000,10.5326,1.3000,-6.5000,0.0000,3.1416},
				{0.0200,30.9040,4.9400,10.5560,1.1700,-6.5000,-0.0000,10.5560,1.1700,-6.5000,0.0000,10.5560,1.1700,-6.5000,-0.0000,3.1416},
				{0.0200,30.8832,4.9400,10.5768,1.0400,-6.5000,0.0000,10.5768,1.0400,-6.5000,0.0000,10.5768,1.0400,-6.5000,0.0000,3.1416},
				{0.0200,30.8650,4.9400,10.5950,0.9100,-6.5000,-0.0000,10.5950,0.9100,-6.5000,0.0000,10.5950,0.9100,-6.5000,-0.0000,3.1416},
				{0.0200,30.8494,4.9400,10.6106,0.7800,-6.5000,0.0000,10.6106,0.7800,-6.5000,0.0000,10.6106,0.7800,-6.5000,0.0000,3.1416},
				{0.0200,30.8364,4.9400,10.6236,0.6500,-6.5000,0.0000,10.6236,0.6500,-6.5000,0.0000,10.6236,0.6500,-6.5000,0.0000,3.1416},
				{0.0200,30.8260,4.9400,10.6340,0.5200,-6.5000,0.0000,10.6340,0.5200,-6.5000,0.0000,10.6340,0.5200,-6.5000,0.0000,3.1416},
				{0.0200,30.8182,4.9400,10.6418,0.3900,-6.5000,-0.0000,10.6418,0.3900,-6.5000,0.0000,10.6418,0.3900,-6.5000,-0.0000,3.1416},
				{0.0200,30.8130,4.9400,10.6470,0.2600,-6.5000,0.0000,10.6470,0.2600,-6.5000,0.0000,10.6470,0.2600,-6.5000,0.0000,3.1416},
				{0.0200,30.8104,4.9400,10.6496,0.1300,-6.5000,0.0000,10.6496,0.1300,-6.5000,0.0000,10.6496,0.1300,-6.5000,0.0000,3.1416},
				{0.0200,30.8104,4.9400,10.6496,0.0000,-6.5000,-0.0000,10.6496,0.0000,-6.5000,0.0000,10.6496,0.0000,-6.5000,-0.0000,3.1416},
				{0.0200,30.8130,4.9400,10.6522,0.1300,6.5000,650.0000,10.6470,-0.1300,-6.5000,0.0000,10.6522,0.1300,6.5000,650.0000,3.1416},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}