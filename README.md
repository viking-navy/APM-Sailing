// Peter Sjolander 2014
 unsigned long pete_wind_time_start = 0;
 unsigned long pete_depth_time_start = 0;
  int Wind_Direction = 0;
  int Water_Depth = 0;
  int wind_to_boat_heading = 0;   // wind vane pete
/*
For example, to set IO pin 17 as an output pin we would now do this:
  hal.gpio->pinMode(17, GPIO_OUTPUT);
*/
// from log
/*
  uint16_t sonar1_distance;
    uint16_t sonar2_distance;
    uint16_t detected_count;
   ///////////////
        sonar1_distance : (uint16_t)sonar.distance_cm(),
        sonar2_distance : (uint16_t)sonar2.distance_cm(),
        detected_count  : obstacle.detected_count,
*/
 uint16_t pete_sonar1_distance;
    uint16_t pete_sonar2_distance;
    uint16_t pete_detected_count;
/*
Reply by Matt Hansen yesterday
I was able to come up with a solution to the problem but it is definitely not a conventional method.  
Since A0 is already set up to read the sonar senor if enabled, I enabled the pin and used this code segment:

int16_t temp_alt = sonar->read();
if(temp_alt > 200) {
do_loiter_time();

I chose 200 (cm) after using the test terminal in MP and seeing that the output was ~31 cm when the RPi was connected to pin A0 
outputting a low and ~257 cm when the RPi found the object and was outputting a high (3.3V).


*/

/*  this sort of works
// sonar testing
int sonar_running_total = 0;
const int max_sonar_smooth = 5;
int sonar_readings[max_sonar_smooth] ;
int aaa = 0;
int water_depth(int sonar)
{
  sonar_running_total += sonar;
  sonar_running_total -= sonar_readings[aaa];
  sonar_readings[aaa] = sonar;
     
  if (aaa < max_sonar_smooth){aaa++;} else {aaa = 0;}
  return	sonar_running_total / max_sonar_smooth;
}

*/

 bool pete_first_time = (true);
#include <string.h>
//String launch_locations[2] = {"Home", "Laguna Lake"};

//String stringOne, stringTwo;
//  stringOne = String("this");
//  stringTwo = String("that");
  

  // on the boat only side.. move once then leave it there
int pete_time_in_tenth_of_seconds()
{
  unsigned long pete_temp_time = millis();
  int pete_time = pete_temp_time / 100;
  return	pete_time;
}




// above this line only on boat not pc


double pete_lat = 33.9084705; // these need to be loaded on the boat
double pete_lng = -117.9361083;


// ||||||||||||||/\/\/\/\/\/\/\/\/\/\|||||||||||||||/\/\/\/\/\/\/\/\
// 

// big and small on capsize day
//# 33.9065467,  33.9080125
//# -117.9370807,  -117.9378596
// ||||||||||||||/\/\/\/\/\/\/\/\/\/\|||||||||||||||/\/\/\/\/\/\/\/\
// rocks and reeds
//---------------------
// boat does not know about this library #include <iostream>
// boat does not know about this library #include <cmath>

double Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}


// so we have room for fifty rocks
// example  int array[100] = {-1};
// first entry is the bunch of reeds along the west elbo of the lake
// I set the lat lng back on the road so the radius would cover all the reeds
// these 3 lines sets up size of the array(50) and adds one entry
double rocks_and_reeds_lat[50]= {33.9084705};
double rocks_and_reeds_lng[50]= {-117.9372635};
int    rocks_and_reeds_safe_distance[50]= {150};  // 150 feet ... about 20 feet south on the south side
	int rocks_and_reeds_total_entries = 1;
	// returns   0 means no danger , any number and is the distance inside the boundry
double look_for_danger (double lat, double lng)
{
	int x = 0;
	while (x < rocks_and_reeds_total_entries)
	{
	  int feet_distance = int((Distance(rocks_and_reeds_lat[x],  rocks_and_reeds_lng[x], 
			 lat,  lng		)) * 363858.97);
			 //Length Of A Degree Of Latitude In Feet  = 363858.97
					  int y = int(feet_distance - rocks_and_reeds_safe_distance[x]);
	  if (y < 0)	  {return y;} /// Danger Danger
	return  feet_distance;
	  x++;
	}  // end of while x

return 0;  // no danger from rocks_and_reeds
} // end of look for danger


// my calculations should agree with above but they dont
//                     Specific Latitude  = 33
//Length Of A Degree Of Latitude In Feet  = 363858.97
//Length Of A Degree Of Longitude In Feet = 306604.32
// from http://www.csgnetwork.com/degreelenllavcalc.html


int add_current_location_to_rock_and_reeds_table()		   
		   {
			rocks_and_reeds_lat[rocks_and_reeds_total_entries]= pete_lat;
             rocks_and_reeds_lng[rocks_and_reeds_total_entries]= pete_lng;
             rocks_and_reeds_safe_distance[rocks_and_reeds_total_entries]= 10;  // ten feet stand off seems fine
	      rocks_and_reeds_total_entries++;
		  if (rocks_and_reeds_total_entries > 49){rocks_and_reeds_total_entries = 47;}
			return rocks_and_reeds_total_entries;
		   }              

// rocks and reeds end of code 
// ||||||||||||||/\/\/\/\/\/\/\/\/\/\|||||||||||||||/\/\/\/\/\/\/\/\
// ||||||||||||||/\/\/\/\/\/\/\/\/\/\|||||||||||||||/\/\/\/\/\/\/\/\

//rr81

 bool first_time = (true);
int pete_roll_at_start_of_tack;

bool shallow_water = false;
int deep_to_shallow = 0;
int shallow_to_deep = 0;
float pete_south_fence = 0;
float pete_north_fence = 0;
int front_sail_servo_ss;
int rear_sail_servo_ss;
int front_sail_servo_ud;
int rear_sail_servo_ud;
int wind_heading;
int straight_sailing_count = 0;
int hold_rudder;
int hold_rud;
int hold_fud;

int reefed = 0;
int south_fence_count = 0;
int north_fence_count = 0;

bool waiting_to_launch = true;

int  end_tack = 0;  // false
int tacking_to_port = 1; // true;
int degrees_from_center_of_boat;

int front_sail_degrees_from_center_of_boat;
int	rear_sail_degrees_from_center_of_boat;

int boat_heading = 180;
int wind_heading_set = 0;

 int tack_direction_at_start_of_tack;
 
  int tack_cnt;
 
int final_after_tack_heading;
 
 
int fss;
int rss;
int fud;
int rud;
int brss;
int frss;

 
int manual_control_time_total = 0;
int time_of_capsize_control_total = 0;
int capsize_control_count = 0;
unsigned long day_sail_start = 0;
unsigned long day_sail_end = 0;
int           day_sail = 0;
unsigned long next_timed_tack = 200000;  // 200 seconds after launch
int tack_reason = 66; // 66 stands for range finder
 

int front_sail_servo_ss_hold;
int  rear_sail_servo_ss_hold;
int front_sail_servo_ud_hold;
int  rear_sail_servo_ud_hold;

int after_tack_clean_up = 0;
int desired_boat_heading_before_tack;

int front_sail_servo_ss_end_tack;
int  rear_sail_servo_ss_end_tack;

int previous_degrees_into_tack;
int previous_range_finder = 0;;

int  rear_sail_servo_ss_adj;
int front_sail_servo_ss_adj;

int rear_sail_servo_ud_adj;

int leave_everything_alone_count = 0;

unsigned int wind_vane;

int tack_for_speed = 0;

int after_tack_recovery_count = 0;
int hold_desired_boat_heading;
      
int previous_boat_heading;

float pete_launch_latitude = 0;
float pete_launch_longitude = 0;
float lat_seconds_from_launch = 0;
float lon_seconds_from_launch = 0;
int pete_dt_home_lat  = 0;
int pete_dt_home_lat_ft  = 0;

int pete_off_heading;
 float distance_to_south_fence;
int lat_tack_counter = 0;
int hold_c3;
// footy prints
int   manual_rssud = 1501;
int   manual_fssud = 1502;
int   manual_fssss = 1503;
int   manual_rssss = 1504;
int   manual_wind_direction = -1;
int heading_adjust = 0;
int off_heading_a = 0;
int front_sail_servo_ud_adj = 1000;

int previous_boat_heading_decade;
int current_boat_heading_decade;
int tack_decades;
int  level_boat_under_sail = 200;
int  level_boat_under_sail_counter = 1; // print on first time thru

int pete_wp_dist = 0;
int next_tack_heading = 0;
int manual_reverse_course = 0;
int print_each_second = 0;
int capsize_control_print = 0;
int manual_control_print = 0;
int tack_print = 0;
int sail_state = 0; // 11 = fast sail  69 = capsize control

int hold_roll_at_start_of_tack = 0;
// old int tack_cnt = 0;

 int sail_in_reverse = 0;
  int backup_then_around_then_forward = 0;
  int jibe = 0;
  int ct_jibe = 0;
int passing_thru_the_wind_nose_first = 0;
int passing_thru_the_wind_nose_first_cnt = 0;
int passing_thru_the_wind_butt_first = 0;
int passing_thru_the_wind_butt_first_cnt = 0;

int tack_direction;
int to_wind_at_start_of_tack;
int current_boat_heading_at_start_of_tack;
int degrees_into_tack;

int tack_end_time;
int tack_start_time;
int smallest_tack_time = 1111;
int tack_off_heading;

int front_servo_tack = 0;
int rear_servo_tack = 0;
int degrees_to_end_tack = 0;
unsigned long tack_start = 0;
unsigned long tack_end = 0;
int tack_time = 0;
int degrees_to_wind = 0;

long randNumber;
int  one_minute = 0;
///////////// sail stuff ////////////////
int sail_show = 1000;
int fast_sail = 0;
int stab = 2;
int range_finder = 0;
// old ... int tack = 0;
int before_tack_heading = 0;
int tack_type = 0; // old
int tack;
int x_degrees = 90;
int level_boat = -600; // for boat with 2 half sails 2014

int current_sailing_spot = 0;
float distance_from_launch;
float pete_current_latitude;
float south_shore[2] = {33.8650000, 33.9088322}; // petes back fence and laguna lake south shore
int south_fence_break = 0;



int bump_servo = 5; 
int previous_ahrs_roll_sensor = 0;
int pete_pitch = 0;
int diff_pitch = 0;
int diff_roll = 0;
int highest_roll = 20;
int previous_pitch = 0;
int lowest_pitch = 10;
int highest_pitch = 10;
int a_pitch = 0;
int pete_roll = 0;
int previous_roll = 0;

int lowest_roll = 10;
int auto_capsize_limit = 0;

//int wind_heading = 90;
int off_heading1;
int off_heading2;
int sail_point_1;
int sail_point_2;
int wind_heading_count = 0;
int wind_heading_total = 0;
 int wind_heading_array[36]; // a count of all the different heading of the wind in tens of degrees
int wind_heading_index = 0;

int wind_direction = -1; // one side of the boat or the other ... -1 means wind from the right
int previous_wind_direction = 1;
int current_wind_direction_count = 0;
int wind_direction_before_tack = 1;
int after_tack_heading = 180;
int off_heading = 3;
int previous_off_heading = 3;
int off_high = 0; 
int off_low = 0;
int off_course = 8;
int previous_off_course = -2;
//int off_wind = 150;
int heel;
int heel_for_fastest_speed_today;
int boat_yaw;
int current_boat_heading; // = 360;  // my home test bed
int desired_boat_heading; // = 360;
int a_roll; // absolute value of roll sensor

int tack_heading_bump_degrees;
int tack_heading_bumps_limit;
int tack_heading_bumps = 0;

int pete_gps_course;
int pete_ground_course_d;
int pete_gps_speed = 0;
int fastest_speed_today = 80; // 

int wind_drift;
int previous_pete_gps_speed;
int max_speed = 50; 
 
int    speed_difference = 2;  // need to set these to soft values 
int     fastest_speed_up = 2;  // so we don't get so many new print outs 
int     fastest_slow_down = 2; // at the start up each time.

int auto_tack_count = 0;
int   boat = 0; // used to reverse servos on the boat that are different than the servos on my test setup.

int           manual_tack_time = 0;
unsigned long manual_tack_start = 0;
unsigned long manual_tack_end = 0;


int            manual_control_time = 0;
unsigned long manual_control_start = 0;
unsigned long manual_control_end = 0;

bool tacking;
bool wind_shift;

///////////////////////////////////////////  ss stands for side to side
int front_sail_servo_ss_center = 1500;
int rear_sail_servo_ss_center = 1500;
//int front_sail_servo_ss = 1500;
//int rear_sail_servo_ss = 1500;
///////////////////////////////////////////  ud stands for up down
int front_sail_servo_ud_center = 1500;
int rear_sail_servo_ud_center = 1500;
//int front_sail_servo_ud = 1500;
//int rear_sail_servo_ud = 1500;
int front_sail_servo_ud_mid = 1250;
int rear_sail_servo_ud_mid = 1250;

 int reef_s_count = 0;
 int reef_s_total = 0;
 int fastest_reefed_speed = 0;

 int before_reef_front_sail_servo_ud;
  int before_reef_rear_sail_servo_ud;
   


int tack_bump = 70;
int tack_counter = 0;
int sailing_counter = 0;
int heading_adjustment = 0;
int heel_adjustment = 0;
int compass_heading = 0; // 0 - 360
int heading_before_tack = 0;
unsigned long start_time_for_capsize_control = 0;
unsigned long end_time_for_capsize_control = 0;
unsigned long time_of_capsize_control = 0;

 unsigned long start_time; 
 unsigned long end_time; 

 unsigned long difference_time = 0;
unsigned long time_out_of_radio = 0;

//                                      00,   10,   20,   30,   40,   50,   60,   70,   80,   90,  100,  110,  120,  130,  140,  150,  160,  170,  180,  190,  200  tack degrees / below servo positions
int tack_left_front_side_to_side[] = {1600, 1500, 1400, 1300, 1200, 1100, 1000, 1000, 1000, 1000, 1000, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900}; 
int tack_left_rear_side_to_side[] =  {1400, 1500, 1600, 1700, 1800, 1900, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 1900, 1900}; 

int tack_right_front_side_to_side[] ={1400, 1500, 1600, 1700, 1800, 1900, 2000, 2000, 2000, 2000, 2000, 2000, 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100}; 
int tack_right_rear_side_to_side[] = {1600, 1500, 1400, 1300, 1200, 1100, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1100, 1100}; 

           int tack_front_up_down[] ={1400, 1300, 1200, 1100, 1100, 1100, 1100, 1100, 1100, 1200, 1300, 1400, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; 
           int tack_rear_up_down[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1400, 1300, 1200, 1100, 1100, 1100, 1200, 1300, 1400, 1400, 1400, 1500}; 
// tack_left_front_side_to_side[]
// tack_left_rear_side_to_side[]
//tack_right_front_side_to_side[]
// tack_right_rear_side_to_side[]
// tack_front_up_down[]
// tack_rear_up_down[]

// could never get this to work
int rear_sail_servo_ss_degrees[] = {140, 140, 130, 120, 110, 100,  90,  80,  70,  60,  40,  30,  15,   5,  -5, -10, -15, -20, -45, -35};  
int front_sail_servo_ss_degrees[] =  {0,   0,   0,   0,   0,   0,   0, -25, -35, -35, -35, -35, -45, -25, -15, -10, -15, -20, -45, -35};  
  
int front_sail_servo_ud_degrees[] = {  0,  0,   0,   0,   0,   0,  25,  25,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  35,  30};  
int rear_sail_servo_ud_degrees[] = {45, 45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  40,  38,  35,  32,  30};  
 

// if we wish to double the size of the tack tables
// all we would need to do is double the tables 
// and change the return line below to: return (degrees_into_tack/5);






//                          example use : front_sail_servo_ss_degrees[tack_table(x)],
int tack_table(int degrees_into_tack)
{
	if (degrees_into_tack < 0)
	{
		degrees_into_tack = 0; //?????????????????????????? hack
		// degrees_into_tack = abs(degrees_into_tack); no effect
	}
	if (degrees_into_tack > 190){degrees_into_tack = 190;}
	 return (degrees_into_tack/10);
}



	









//==============================================//// 0 is down,  45 is full up
// converts degrees_of_sail_up_down from table to servo numbers...  0 = all the way down, 45 = all the way up
// an up-down(ud) sail servo function
int up_down_from_angle (int degrees_of_sail_up_down) 
{ 
	int   ud_servo_position = (degrees_of_sail_up_down * 11) + 1000;
    return  ud_servo_position;
}

//$servo_position = int(($degrees_from_center_of_boat * 1000/90) + 1500);


	 

	  int limit_heading_change(int x)
	  {
		if (x > 180){x = x - 360;} if (x < -180) {x = x + 360;} return x;
	  }
	  
int to_desired_boat_heading (int current_heading)
	{
	  int x = desired_boat_heading - current_heading; x = limit_heading_change(x); return x;
    }

int to_wind (int current_heading)
    {
	  int x = wind_heading - current_heading; x = limit_heading_change(x); return x;
    }
int off_wind;                         
   

// 
//int degrees_into_tack;
int find_diff (int starting, int current, int td)
{ 
	int dsc;
	if (td < 0)
	{
		if (starting < current)
		{
			dsc = 360 + starting - current;
		}
		else
		{
			dsc = starting - current;
		}
	}
	else
	{
		if (starting > current)
		{
			dsc = 360 +  current - starting;
		}
		else
		{
			dsc = current - starting;
		}
	}
//	cout  << current<< "\t" << dsc << "\t\t\t\t\t\t\t\t\t\t" <<  td << "\t" << starting << "\t"  <<endl;
        if (dsc == 359){dsc = 0;}
	if (dsc > 190){dsc = 190;}
    return dsc;
} // end of  find_diff 





 














// above from c++
//
// this page was created by Peter Sjolander
// use at your own risk.  I am very new to all this Nov. 2013
// the sail boat I am testing on is a Viking ship
// the sails are my design
// I steer the boat with the sails
// I have no rudder
// I have no wind vane

// my ship looks like this from the side

//           /\\           /\\
//          /  \\         /  \\
//         /     \\      /    \\
//       /        \\    /      \\
//     /===========\s\ /========\s\
//           ==s===========--s
//          //////////////
//





//MSG, Failsafe - Long event on, 
//MODE, RTL, 11
//MODE, RTL, 11
  int rtl_cnt = 0;
  
  int  print_at_one_second = 0;
  
 float   pete_gps_latitude;
 float   pete_gps_longitude;
 
 
 
unsigned long total_speed_we_bought = 0;
unsigned long level_boat_under_sail_avg= 0;
unsigned long level_boat_under_sail_total= 0;
 
 
 


 float south_lat_limit;
 
int south_limit = 0;
float laguna_lake_south_lat = 33.9085544;

// -117.9378650  -117.9370698 high and low longatude at laguna lake on day of capsize
//capsize day north wind 33.9065981, -117.9378650,     33.9078771, -117.9370698,
// normal day west wind 33.9093500, -117.9345557,    33.9085734, -117.9362308,
// if we run a line between these two days we could stay out of the reeds
//    


   float tack_longitude[200]; 
   float tack_latitude[200];
     int tacking_reason[200];
     int total_tacks = 0; // index limit for above array 
 
     
// ??????????? have_position = ahrs.get_position(&current_loc);

  
 
 
 
 
 
 
 
 
 
 
 
//////////// start of pete patch for total servo control
///  I pulled this out of some servo sample code

void multiread(AP_HAL::RCInput* in, uint16_t* channels) {
   // Multi-channel read method: 
    uint8_t valid;
    valid = in->read(channels, 8);
  
 //   hal.console->printf_P(
 //           PSTR("multi      read %d: %d %d %d %d %d %d %d %d\r\n"),
 //           (int) valid, 
 //           channels[0], channels[1], channels[2], channels[3],
 //           channels[4], channels[5], channels[6], channels[7]);  
         
}


void multiwrite(AP_HAL::RCOutput* out, uint16_t* channels) 
{
  // used to reverse servos on the boat that are different than the servos on my test setup. rear side to side servo channels[3] =  rear_sail_servo_ss; 
  if (boat == 1)   {channels[3] = (2000 - channels[3]) + 1000;}
  
    out->write(0, channels, 8);
    // Upper channels duplicate lower channels
    out->write(8, channels, 8);
}


///////////////////////////////////////////// end pete servo patch


 








////////////////// this is my first function.  it keeps the heading between 0 and 360
// to use ... current_heading = wrap_heading(current_heading);
int wrap_heading (int current_heading)
{
  int wrapped_heading = current_heading;
  if (current_heading > 360) {wrapped_heading = current_heading - 360;}
  else
  {if (current_heading < 0) {wrapped_heading = 360 + current_heading;}}
  return wrapped_heading;
}
////////////////// end of .... this is my first function.  it keeps the heading between 0 and 360
////////////////////////////  but maybe it should keep the               values between 0 and 359 ????????????





int upwind (int x_degrees) 
{
 int  upwind_x =  current_boat_heading + (x_degrees * wind_direction);
  int upwind_h = wrap_heading ( upwind_x );
   return upwind_h;
}




int calc_off_heading (int ddesired_boat_heading,int ccurrent_boat_heading)
{
  int off_heading = ddesired_boat_heading - ccurrent_boat_heading;
 int fix_off_heading = off_heading;
  if (off_heading >=  180) {fix_off_heading = off_heading - 360;}
  if (off_heading <= -180) {fix_off_heading = off_heading + 360;} 
  return fix_off_heading;
 }



int find_wind_direction (int roll)
{
    if( roll < 0) {wind_direction = -1;} else {wind_direction = 1;} // that is one side of the boat or the other
    return wind_direction;
}










int xx = 0;



//  capsize_control  ////  capsize_control  ////  capsize_control  ////  capsize_control  ////  capsize_control  ////  capsize_control  ////  capsize_control  //




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
////////////////////////////////////////////  capsize_control  //////////////////////////////////////////////////////////////////////// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
  // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
// called by the main loop ten times a second  
void capsize_control(void)
{
  
   
      // read the 8 ch s from the RC
   uint16_t channels[8];
   hal.gpio->write(27, 1);
   multiread(hal.rcin, channels);
   
 //   range_finder =  rf->read();  /// petes sonar is pointed straight forward , in centameters I think

 
 
 
 //if (Raw_Wind_Direction > .01) {Wind_Direction = (Raw_Wind_Direction / 5) * 360;}
 // int peteWind_Direction = Wind_Vane_pin->pulsewidth(); // pete
 
 current_boat_heading = ahrs.yaw_sensor / 100;  // use while tacking because the gps is not good then.  not enough forward speed.

   
   if (sailing_counter < 16){return;}

   // servos seem to be a distraction ...  front_sail_servo_ud, rear_sail_servo_ud,   front_sail_servo_ss, rear_sail_servo_ss
 // print_each_second = 1; // pete to print fast
   if (print_each_second == 1)
   {
 gcs_send_text_fmt(PSTR("sail,%i,%i,%i,%i,%i,%i,%i,%i"),sail_state,wind_to_boat_heading,Water_Depth,range_finder,desired_boat_heading,current_boat_heading,tack_off_heading,tack_heading_bumps);
 print_each_second = 0;
   }
 
 // this needs  to be in a function called by the main loop very often
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
     ////////////  capsize control  ... lower the sails and center them for 5 seconds
    // if you carry the boat by one oar sail, the sails will stay down and centered the whole time 
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////     
  
    
    





     
    
    pete_gps_course =  g_gps->ground_course_cd;   // it looks like the cd stands for 1/100 degrees
  //  boat_yaw = ahrs.yaw_sensor / 100;  // use while tacking because the gps is not good then.  not enough forward speed.
  //  compass_heading = boat_yaw;
    pete_ground_course_d = pete_gps_course / 100;  // this works fine and gives numbers from 0 to 360
    wind_drift = compass_heading - pete_ground_course_d;
   // current_boat_heading = compass_heading; // was ... pete_gps_course / 100; // gives numbers from 0 to 360
  
    off_wind = calc_off_heading(wind_heading, current_boat_heading);
   
    previous_pete_gps_speed = pete_gps_speed;
    
    pete_gps_speed =  g_gps->ground_speed_cm; //  / 10;   /// pete test // 1 centimeter / second = 0.0223693629 miles per hour
 
    
    pete_roll =  ahrs.roll_sensor  - level_boat;
     heel = abs( pete_roll);
    
  
     
   
     
    if (heel > 7500)    // new purple foam
    {     
      if (end_time_for_capsize_control == 0) 
      {
      start_time_for_capsize_control = millis();  
      capsize_control_count++;
      gcs_send_text_fmt(PSTR("c c start"));
      gcs_send_text_fmt(PSTR("%i"),capsize_control_count);
     
       

      print_at_one_second = 1;
      
      
      }
    end_time_for_capsize_control = millis() + 5000;  // keep the sails down for 5 seconds
    }
    
    
     sail_state = 69;
      
    
    
    if (end_time_for_capsize_control > millis())
  {
    //                                                                                if tack switch while capsized record rock????????????????????????
  channels[0] = 1000;   channels[1] =  1000; // lower both sails
  channels[2] = 1500;   channels[3] =  1500; // center both sails
  multiwrite(hal.rcout, channels);  //  write to servos
  return;
  }
  //////////////// end of first part of capsize control /////////////////////////
    if (end_time_for_capsize_control == 0) {}
    else 
    {
    int  time_of_capsize_control = (end_time_for_capsize_control - start_time_for_capsize_control) / 1000;  // gives delay in seconds
      end_time_for_capsize_control = 0;
      start_time_for_capsize_control = 0;
      gcs_send_text_fmt(PSTR("c c end"));
    gcs_send_text_fmt(PSTR("%i"), time_of_capsize_control);
    
    time_of_capsize_control_total = time_of_capsize_control_total + time_of_capsize_control;
    gcs_send_text_fmt(PSTR("c c t tot"));
    gcs_send_text_fmt(PSTR("%i"), time_of_capsize_control_total);
    }
      //////////// end  capsize control     logic
    ///////////////////////////////////////////////////////////////////////
    
    
    
    
    
    
    
    
    
   // yesterday I had 5 capsize control events and that caused me to write this little reef code
  // I may need to alter this  by using the amount over 20 degrees to alter the reef amount ??????????????????????????
  // for example 1 degree over 20 could give 20% down while 4 degrees over could give 80% down...  deg_over_20 = (heel - 2000) / 100;
     
    if (heel > 5500)    // new purple
    {     
    // gcs_send_text_fmt(PSTR("reef s"));
    // gcs_send_text_fmt(PSTR("%i"), pete_gps_speed);
        reef_s_count++;
        reef_s_total += pete_gps_speed;
     
    if (pete_gps_speed > fastest_reefed_speed)
       { 
       fastest_reefed_speed = pete_gps_speed;
       }
    if (reefed == 0)  
   { 
    before_reef_front_sail_servo_ud = front_sail_servo_ud;
    before_reef_rear_sail_servo_ud = rear_sail_servo_ud;
    reefed = 1;
   }
     sail_state = 99;
     channels[1] = ((front_sail_servo_ud - 1000) / 2) + 1000;  // reef the front sail by 1/2
     channels[0] = (( rear_sail_servo_ud - 1000) / 3) + 1000;  // reef the rear sail by 1/3  to turn a little into the wind 
     front_sail_servo_ud = channels[1];  
     rear_sail_servo_ud  = channels[0];
     multiwrite(hal.rcout, channels);  //  write to servos
     return;
    }
    
   
     
    
   
    
    // check speed ... if we are moving and speeding up ... leave_everything_alone
    if (( previous_pete_gps_speed < pete_gps_speed) && (pete_gps_speed > 50))
    {
      total_speed_we_bought = total_speed_we_bought + ( pete_gps_speed - previous_pete_gps_speed );
       leave_everything_alone_count++;
      // gcs_send_text_fmt(PSTR("leave_everything_alone"));  
      // gcs_send_text_fmt(PSTR("%i"), leave_everything_alone_count);
      // gcs_send_text_fmt(PSTR("%i"), previous_pete_gps_speed);
      // gcs_send_text_fmt(PSTR("%i"), pete_gps_speed);
           // gcs_send_text_fmt(PSTR("%i"), total_speed_we_bought);
   channels[0] =  rear_sail_servo_ud;
   channels[1] = front_sail_servo_ud;  
   channels[2] = front_sail_servo_ss;
   channels[3] =  rear_sail_servo_ss; 
  
          multiwrite(hal.rcout, channels);  //  write to servos
       return;
    }
    
    
   
 // when we get here after reefing unreef
    if ( reefed == 1)
    {
    reefed = 0;
    front_sail_servo_ud = before_reef_front_sail_servo_ud;
    rear_sail_servo_ud = before_reef_rear_sail_servo_ud;
    }
  
    
    
   
    //  manual control  manual control  manual control  manual control  manual control  manual control  manual control  manual control  manual control  manual control
    
    
      ///////////////////////////////////////////////////////////////////////////////////////////////////
     ////////////////////////////////   manual control /////////////////////////
      /////////////////////////////////////////////////////////////////////////////////////
            if (channels[5] > 1500)  // this is ch 6 on the RC    for manual control
            {     
              tack = 0; tack_cnt = 0; // reset for a tack just to be on the safe side
               end_tack = 0;
              previous_degrees_into_tack = 0;
                        degrees_into_tack = 0; // safe guard
              if (manual_control_start == 0)
              {
                manual_control_start = millis(); 
                gcs_send_text_fmt(PSTR("m c start"));
               }
         
           sail_state = 88;
           
           
            // for testing at home uncomment ... pete_gps_speed = 35;
    
    
    
    
    
     
     // works ... but need to keep moving the joystick back and forth after each tack if you go to manual control
  
  
 // ?? if ((channels[2] > 1500) && (front_sail_servo_ss < 1500)){front_sail_servo_ss = 1500 - (channels[2] - 1500);} // reverse the input


       front_sail_servo_ss = channels[2]; // + hold_c3; 
       rear_sail_servo_ss = channels[2]; // - hold_c3; 
       hold_c3 = channels[3] - 1500;
       front_sail_servo_ss = front_sail_servo_ss  + hold_c3;
       rear_sail_servo_ss = rear_sail_servo_ss  - hold_c3;
       
       channels[2] = front_sail_servo_ss;
       channels[3] = rear_sail_servo_ss;
       
         rear_sail_servo_ud = channels[0];
       front_sail_servo_ud = channels[1];  
  
   
    // save the manual servo positions for the automatic control ... this may be a bad idea .. I think I am triping over this
       //  manual_rssud = channels[0];
       //  manual_fssud = channels[1];
       //  manual_fssss = channels[2];
         //manual_rssss = channels[3];
         //manual_wind_direction = wind_direction;
         desired_boat_heading = current_boat_heading;
         straight_sailing_count = 0;
         
    multiwrite(hal.rcout, channels);  //  write to servos
            
       
             return;
            }
    //////////////////////////////// end   manual control /////////////////////////
     if (manual_control_start == 0) {} else
              {
                manual_control_end = millis();
                manual_control_time = ( manual_control_end - manual_control_start) / 1000;
                gcs_send_text_fmt(PSTR("m c end"));
                 gcs_send_text_fmt(PSTR("%i"), manual_control_time);
                 manual_control_time_total = manual_control_time_total + manual_control_time;
                
                 
                manual_control_start = 0; 
                manual_control_end = 0;     
                desired_boat_heading = current_boat_heading;  // set all the time while manual so ready for automatic
              }  
   //////////////////// end of manual control end process  ////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////  
    
      
   //  end of manual control end of manual control end of manual control end of manual control end of manual control end of manual control end of manual control
    
    
    
    
    
    
   //  if ((range_finder < 20) && (previous_range_finder > 20) && (tack == 0)) 
  //  {tack = 1; tack_reason = 77;  gcs_send_text_fmt(PSTR("shallow"));} // check for shallow water

//                                        just for testing  
// the sonar I have is MB7067 XL-MaxSonar-WRC1
 //      if ((range_finder < 20) && (previous_range_finder > 20)) 
   //    {gcs_send_text_fmt(PSTR("shallow"));} // check for shallow water
  
  
      if ((current_loc.lng < pete_south_fence) && (current_boat_heading > 90) && (current_boat_heading < 270) && (tack == 0))
    //   {tack = 1; tack_reason = 66;  gcs_send_text_fmt(PSTR("south fence"));} // check for south fence
      {south_fence_count++;} // check for south fence
        
     if ((current_loc.lng > pete_north_fence) && (current_boat_heading >270) && (current_boat_heading < 90) && (tack == 0))
      // {tack = 1; tack_reason = 65;  gcs_send_text_fmt(PSTR("north fence"));} // check for south fence
         {north_fence_count++;} // check for north fence
       
   if ((channels[4] > 1600) && (tack == 0)) 
      {tack = 1; tack_reason = 99; gcs_send_text_fmt(PSTR("manual tack"));} // check for manual tack switch
    
// five points of tack
if (tack == 1)
	{
	  tack_start_time = pete_time_in_tenth_of_seconds(); 
	  gcs_send_text_fmt(PSTR("start t"));
          tack_heading_bump_degrees = 10; //15;
          tack_heading_bumps_limit = 19; //16;
          tack_heading_bumps = 0; // 1;
          tack = 2; 
          end_tack = 0;
	  current_boat_heading_at_start_of_tack = current_boat_heading;
	  // desired_boat_heading = after_tack_heading;

          if (rear_sail_servo_ss < 1500)
          {
// to test array comment next line
front_sail_servo_ss = 1000;  rear_sail_servo_ss = 2000; front_sail_servo_ud = 1100; rear_sail_servo_ud = 1500;
          tacking_to_port = 1;
          desired_boat_heading = current_boat_heading - tack_heading_bump_degrees;
          desired_boat_heading = wrap_heading(desired_boat_heading);
          gcs_send_text_fmt(PSTR("left t"));  
          }
          else
          {
// to test array comment next line
front_sail_servo_ss = 2000;  rear_sail_servo_ss = 1000; front_sail_servo_ud = 1100; rear_sail_servo_ud = 1500;
          tacking_to_port = 0;
          gcs_send_text_fmt(PSTR("right t"));  
          desired_boat_heading = current_boat_heading + tack_heading_bump_degrees;
          desired_boat_heading = wrap_heading(desired_boat_heading);
          }

          
	} // end of if (tack == 1) 




if (tack == 2)
{
//  bool xxxx = true;  // just sample
      sail_state = 99; // tacking
      tack_off_heading = calc_off_heading (desired_boat_heading, current_boat_heading);
      
     
      if (tacking_to_port == 1)
      {
       
        if (tack_off_heading > 0)
          {
          desired_boat_heading = current_boat_heading - tack_heading_bump_degrees;
          desired_boat_heading = wrap_heading(desired_boat_heading);
          tack_heading_bumps++;
// sail position array from 0 to 20 which equals 00 to 200 degrees
// left is to port
       //  front_sail_servo_ss = tack_left_front_side_to_side[tack_heading_bumps];
     //    rear_sail_servo_ss =  tack_left_rear_side_to_side[tack_heading_bumps];
   //      front_sail_servo_ud =  tack_front_up_down[tack_heading_bumps];
 //        rear_sail_servo_ud =  tack_rear_up_down[tack_heading_bumps];
     //     if (tack_heading_bumps == 17) {front_sail_servo_ss = 1400;}
         
      //     gcs_send_text_fmt(PSTR("sail,%i,%i,%i,%i,%i,%i,%u,%i"),sail_state,desired_boat_heading,current_boat_heading,range_finder,tack_off_heading,tack_heading_bumps,wind_vane,leave_everything_alone_count);

          } else 
        {
        // none
        }
    
      }
      else
      {
       
        if (tack_off_heading < 0)
        {
          desired_boat_heading = current_boat_heading + tack_heading_bump_degrees;
          desired_boat_heading = wrap_heading(desired_boat_heading);
          tack_heading_bumps++;

// sail position array from 0 to 20 which equals 00 to 200 degrees
// right is to starboard
        // front_sail_servo_ss = tack_right_front_side_to_side[tack_heading_bumps];
      //   rear_sail_servo_ss =  tack_right_rear_side_to_side[tack_heading_bumps];
    //     front_sail_servo_ud =  tack_front_up_down[tack_heading_bumps];
  //       rear_sail_servo_ud =  tack_rear_up_down[tack_heading_bumps];

    //      if (tack_heading_bumps == 17) {front_sail_servo_ss = 1600;}
         
      //     gcs_send_text_fmt(PSTR("sail,%i,%i,%i,%i,%i,%i,%u"),sail_state,desired_boat_heading,current_boat_heading,range_finder,tack_off_heading,tack_heading_bumps,wind_vane);

        } else   
        {
        // none
        }
        
      }  
      
//  gcs_send_text_fmt(PSTR("sail,%i,%i,%i,%i,%i,%i,%i"),sail_state,desired_boat_heading,current_boat_heading,range_finder,tack_off_heading,tack_heading_bumps,tack_reason);
       
       if (tack_heading_bumps < 18)   // less than so that if we break out of the tack we have a good number
       {
       final_after_tack_heading = current_boat_heading;
   //    gcs_send_text_fmt(PSTR("final_after_tack_heading,%i"),final_after_tack_heading);
       }
       
       // does not stay   
     if (tack_heading_bumps == 13) {front_sail_servo_ss = 1500;}
       if (tack_heading_bumps == 10) // 90 degrees from start of tack
       {
       wind_heading = current_boat_heading;
       front_sail_servo_ud = 1500; rear_sail_servo_ud = 1100;
     //  gcs_send_text_fmt(PSTR("wind_heading,%i"),wind_heading);
       }
       // now check if the tack is done  
       next_timed_tack = pete_time_in_tenth_of_seconds() + 220;   // sometimes my tacks are so slow that the timer runs out while still tacking
       if ((channels[0] > 1900) && (end_tack == 0)) {end_tack = 1;}
       if ((tack_heading_bumps >= tack_heading_bumps_limit) && (end_tack == 0)) {end_tack = 1;}
     
      
   } // end of if (tack == 2)   
      
  
      
 
       
      if (end_tack == 1) 
            {
               
              end_tack = 0;
              tack_end_time =  pete_time_in_tenth_of_seconds();
              tack_time = tack_end_time - tack_start_time;  // tack time in tenths of seconds
              if (tack_time < smallest_tack_time){smallest_tack_time = tack_time;}
              
              gcs_send_text_fmt(PSTR("end t"));                                 gcs_send_text_fmt(PSTR("%i"),tack_time);
              /*
              gcs_send_text_fmt(PSTR("smallest_tack_time"));                    gcs_send_text_fmt(PSTR("%i"),smallest_tack_time);
              gcs_send_text_fmt(PSTR("current_boat_heading_at_start_of_tack")); gcs_send_text_fmt(PSTR("%i"),current_boat_heading_at_start_of_tack);
              gcs_send_text_fmt(PSTR("wind_heading"));                          gcs_send_text_fmt(PSTR("%i"),wind_heading);
              gcs_send_text_fmt(PSTR("final_after_tack_heading"));              gcs_send_text_fmt(PSTR("%i"),final_after_tack_heading);
              gcs_send_text_fmt(PSTR("current_boat_heading"));                  gcs_send_text_fmt(PSTR("%i"),current_boat_heading);
              gcs_send_text_fmt(PSTR("north_fence_count"));                     gcs_send_text_fmt(PSTR("%i"),north_fence_count);
              gcs_send_text_fmt(PSTR("south_fence_count"));                     gcs_send_text_fmt(PSTR("%i"),south_fence_count);
            
              if (reef_s_count > 0)
              {
              gcs_send_text_fmt(PSTR("reef_s_count"));
              gcs_send_text_fmt(PSTR("%i"), reef_s_count);
              int reef_s_avg = reef_s_total / reef_s_count;      reef_s_count = 0; reef_s_total = 0;
              gcs_send_text_fmt(PSTR("reef_s_avg"));
              gcs_send_text_fmt(PSTR("%i"), reef_s_avg);
              gcs_send_text_fmt(PSTR("fastest_reefed_speed"));
              gcs_send_text_fmt(PSTR("%i"), fastest_reefed_speed);
              }
              
              
              if (leave_everything_alone_count > 0)
              {              
              gcs_send_text_fmt(PSTR("leave_everything_alone"));  
              gcs_send_text_fmt(PSTR("%i"), leave_everything_alone_count);
              //  gcs_send_text_fmt(PSTR("%i"), previous_pete_gps_speed);
              //  gcs_send_text_fmt(PSTR("%i"), pete_gps_speed);
              gcs_send_text_fmt(PSTR("%i"), total_speed_we_bought);
              total_speed_we_bought = 0; leave_everything_alone_count = 0;
              }
                */
	      tack = 0; tack_decades = 0;  straight_sailing_count = 0; tack_heading_bumps = 0;

              // for the purple
              if (rear_sail_servo_ss < 1500) 
              {rear_sail_servo_ss = 1300;}
              else
              {rear_sail_servo_ss = 1700;}
              front_sail_servo_ss = rear_sail_servo_ss;  // my new exit from tack
              desired_boat_heading = final_after_tack_heading;
              
     }   // end of if (end_tack == 1)
 
  

 

 

 
 
   // steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering
   // steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering
   // steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering
   // steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering
   // steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering steering   steering
 
  // my lattest stab at steering 
 // tack = 0;
  if (tack == 0)
  {
  
    straight_sailing_count++;
   
   // if (straight_sailing_count == 1){hold_rudder = rear_sail_servo_ss;}
   if (straight_sailing_count == 1){hold_fud = front_sail_servo_ud; hold_rud = rear_sail_servo_ud;}
    sail_state = 11; // straight sailing
     
    tack_off_heading = calc_off_heading (desired_boat_heading, current_boat_heading);
 
  
   
   if (rear_sail_servo_ss > 1500)
   {   
        rear_sail_servo_ud = hold_rud + (tack_off_heading * 18);
       front_sail_servo_ud = hold_fud - (tack_off_heading * 5);
    }
    else 
    {
        rear_sail_servo_ud = hold_rud - (tack_off_heading * 18);
        front_sail_servo_ud = hold_fud + (tack_off_heading * 5);
      
     
    } 
    
  } // end of if (tack == 0)
  
  
  

    
// jibe    .. lower rear sail half way
// turn both sails out as soon 45 degree away point has been reached..
    
 
       ///////////////// start of  fast sail   /////////////////////////////////
     
              ///////////////// end of  fast sail   /////////////////////////////////

      
     // this is the output for both the tack and straight sail 
      
     
       if (front_sail_servo_ud < 1000){front_sail_servo_ud = 1001;}   if (front_sail_servo_ud > 1500){front_sail_servo_ud = 1499;} 
       if (rear_sail_servo_ud  < 1000){ rear_sail_servo_ud = 1001;}   if (rear_sail_servo_ud  > 1500) {rear_sail_servo_ud = 1499;} 
       
       if (front_sail_servo_ss < 1000){front_sail_servo_ss = 1001;}   if (front_sail_servo_ss > 2000){front_sail_servo_ss = 1999;} 
       if ( rear_sail_servo_ss < 1000){ rear_sail_servo_ss = 1001;}   if ( rear_sail_servo_ss > 2000){ rear_sail_servo_ss = 1999;} 
  
     
    channels[2] = front_sail_servo_ss;
    channels[3] =  rear_sail_servo_ss; 
    channels[1] = front_sail_servo_ud;  
    channels[0] =  rear_sail_servo_ud;
  
     
      multiwrite(hal.rcout, channels);  ////////////  write to servos  //////////////////////////////////
             
     ///////////////// end of  steer in a straight line /////////////////////////////////////
     
     
     
    

             
              
              
  
      
 /////////////////////////////////////////////////////////////////////////
 //  there are a number of ways to start an auto tack
 // 1.  new fastest speed
 // 2.  front distance sensor sees something
 // 3.  bump into something
 // 4.  very quick slow down
 // 5.  past a certain angle from next way point
 // 6.  reached a certain lat. lon. or line
  //////////////////////////////////////////////////////////////////////////   
  
  // this would seek home for now
  pete_wp_dist = wp_distance;
 // if (tack == 0){if (pete_wp_dist > 20){tack = 1; gcs_send_text_fmt(PSTR("x tack s"));}} // x stands for exploring (seems to work feb 14 2014)
 // if (tack == 1){if (pete_wp_dist < 18){tack = 0; gcs_send_text_fmt(PSTR("x tack e"));}}
  
 
  
  
      //   still thinking
    // if a sensor and speed ... tack
    // if low speed  sail backwards
    
    ////////// start sail the boat backwards //////////////////////////////??????????
    
    // front sail down, rear sail centered
    // the boat should weather vane back and forth thru the eye of the wind
    // record the eye of the wind
    // after six back and forths, wait until the boat is pointed away from the direction of the sensor ping
    // front sail up and rear sail down 
    // wait 90 degrees then straight sail
    
   // end  still thinking
    
    
    ////////// end sail the boat backwards //////////////////////////////
    
    
    
    
    
         // need to add in heel and speed feedback adjustment?????????????/
         
         
         
   
    
}    
// end of capsize control and manual control   ... all fast loop stuff














///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////  sailing /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// main ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// this loop is called by the main loop every one second

 void sailing(void)
{
  // stringOne = String("this");
 // stringTwo = String("that");
  
      
   // read the 6 ch s from the RC
   uint16_t channels[8];
   hal.gpio->write(27, 1);
   multiread(hal.rcin, channels);

      
  
  
  
  print_each_second = 1; // this is information for the fast sailing loop to only print one time per second
  
// start_time = millis(); // takes 21 or 31 ms to run

  
    
    sailing_counter++;
    
    /*  // I don't think this works
 //   bool waiting_to_launch = true;
  if (waiting_to_launch)
{
  gcs_send_text_fmt(PSTR("waiting_to_launch"));
   //    pete_launch_latitude = g_gps->latitude; // float
      pete_launch_latitude =   current_loc.lat;
      gcs_send_text_fmt(PSTR("pete_launch_latitude")); gcs_send_text_fmt(PSTR("%f"), pete_launch_latitude);
     //  pete_launch_longitude = g_gps->longitude; // float  
       if (pete_launch_latitude < 33.33)
     {
     sailing_counter = 0; // sailing_counter - 1;
     gcs_send_text_fmt(PSTR("no gps yet"));
   } else {waiting_to_launch = false;}
 }      
 
 */
 
 
   if (sailing_counter == 1)
 {
 day_sail_start = millis(); 
 gcs_send_text_fmt(PSTR("sjolander.com/viking"));
 
float laguna_lake_south_shore = 33.9088322; // %2.7f
gcs_send_text_fmt(PSTR("laguna_lake_south_shore"));
gcs_send_text_fmt(PSTR("%2.7f"), laguna_lake_south_shore);
 } 
   if (sailing_counter == 12)
   {
      
       
       // decide where we are ... home, laguna lake, etc
      // float laguna_lake_south_shore = 33.9088322; // %2.7f
     //  float home_south_shore        = 33.8650000;  // rocking chair = 33.8654305;
       //lat_dist = pete_current_latitude
       
     //  float south_shore[2] = {33.8650000, 33.9088322} 33.9087634
     // char south_shore_name[12][12] = {"Home", "Laguna Lake"};
         //  gcs_send_text_fmt(PSTR("south_shore_name[0]")); 
       // gcs_send_text_fmt(PSTR("%c"),south_shore_name[0][11]); // prints a blank line
   
    // char foobar[2][11];
      
   //   float smallest_distance_from_launch_so_far = .9; // a very big number in this context
  
      gcs_send_text_fmt(PSTR("board power"));
       current_sailing_spot = 0;
       
       for (int i=0; i < 3; i++)
       {
         
        float pete_distance_from_launch = (pete_launch_latitude - south_shore[i]); // * 100000;
        if (pete_distance_from_launch < 0.00001){current_sailing_spot = i;}
        gcs_send_text_fmt(PSTR("%f"),pete_launch_latitude); 
        gcs_send_text_fmt(PSTR("%f"),south_shore[current_sailing_spot]); 
        gcs_send_text_fmt(PSTR("pete_distance_from_launch"));
        gcs_send_text_fmt(PSTR("%f"),pete_distance_from_launch); 
        
        } // end of for loop
        
        
         gcs_send_text_fmt(PSTR("current_sailing_spot")); gcs_send_text_fmt(PSTR("%i"),current_sailing_spot); 
        if (current_sailing_spot == 0){gcs_send_text_fmt(PSTR("1518 W West Ave. Fullerton, Ca.")); }
        if (current_sailing_spot == 1){gcs_send_text_fmt(PSTR("Laguna Lake, Fullerton, Ca.")); }
        
         
// just testing
          float pete_latitude    = current_loc.lat;  gcs_send_text_fmt(PSTR("pete_latitude")); gcs_send_text_fmt(PSTR("%f"),pete_latitude); 

/*
      float pete_longitude   = current_loc.lng;      gcs_send_text_fmt(PSTR("pete_longitude")); gcs_send_text_fmt(PSTR("%f"),pete_longitude); 

float about_fifty_feet_south_of_launch = 0.00050;  // subtract this from the launch lat and you have the south fence
float about_twenty_five_feet_south_of_launch = 0.00025;  // subtract this from the launch lat and you have the north fence

// line 1325:1
//float 
pete_south_fence    = (float)(pete_latitude - about_fifty_feet_south_of_launch);
               gcs_send_text_fmt(PSTR("pete_south_fence")); gcs_send_text_fmt(PSTR("%f"),pete_south_fence); 

pete_north_fence    = (float)(pete_latitude - about_twenty_five_feet_south_of_launch);
               gcs_send_text_fmt(PSTR("pete_north_fence")); gcs_send_text_fmt(PSTR("%f"),pete_north_fence); 




     
     
   int     pete_board_voltage                           = board_voltage();
     gcs_send_text_fmt(PSTR("pete_board_voltage")); 
gcs_send_text_fmt(PSTR("%i"),pete_board_voltage); 
 */    
     
// from log pde
// seems to work ok.  no use for it right now
/*
int pete_ground_speed                                 = (uint16_t)(ground_speed*100);
     gcs_send_text_fmt(PSTR("pete_ground_speed")); 
gcs_send_text_fmt(PSTR("%i"),pete_ground_speed); 

*/



   }


/*
// fence checking  fence checking  fence checking  fence checking  fence checking  fence checking  fence checking  fence checking  fence checking  fence checking 

 pete_current_latitude = g_gps->latitude; // float
    //   pete_current_longitude = g_gps->longitude; // float  
    distance_to_south_fence =  (pete_current_latitude - south_shore[current_sailing_spot]) * 100000;
    if ((south_shore[current_sailing_spot] > pete_current_latitude) && (current_boat_heading > 90) && (current_boat_heading < 270)) 
    {south_fence_break++;}
    else
    {south_fence_break = 0;}    
       
if ((tack == 0) && (south_fence_break > 0))  

{
//tack = 1; sail_state = 95;

}     
       
       
//    done with south fence logic       
       
       */
       
       
   
    // public info found in file...  AP_GPS_HIL.cpp
    pete_gps_course =  g_gps->ground_course_cd;   // it looks like the cd stands for 1/100 degrees
    
    pete_ground_course_d = pete_gps_course / 100;  // this works fine and gives numbers from 0 to 360
    wind_drift = compass_heading - pete_ground_course_d;
 
  previous_pete_gps_speed = pete_gps_speed;
    
    pete_gps_speed =  g_gps->ground_speed_cm; //  / 10;   /// pete test // 1 centimeter / second = 0.0223693629 miles per hour
 
    
    pete_roll =  ahrs.roll_sensor  - level_boat;
     heel = abs( pete_roll);
  
   // it seems to take a while for the roll to get right.
   // we want to do this one second before we raise the sails
    if (sailing_counter == 14)
      {
      gcs_send_text_fmt(PSTR("216 sail version"));  //     /////////////////////////                                                    version control
      gcs_send_text_fmt(PSTR("%f"),south_shore[current_sailing_spot]);
      gcs_send_text_fmt(PSTR("level_boat")); 
      gcs_send_text_fmt(PSTR("%i"),level_boat); 
     }  
     
  // 214 sail version ... I installed the external compass upside down and changed the APM frame type to NO external compass
  // because it seems that the internal compass is upside down and the software to allow for this with the external compass has a bug.
  
  // 204 sail version .. the sonar seems to work.  a bit gittery so I average 5 readings ... have yet to test it in the water.
  
  
     // seems we need to give the roll sensors 15 seconds for the roll to settle in so we get a level starting point
     if (sailing_counter < 15) 
   {
      level_boat = ahrs.roll_sensor; // / 100; //  adjust for not level boat
    //  gcs_send_text_fmt(PSTR("level_boat")); 
    //  gcs_send_text_fmt(PSTR("%i"),level_boat); 
      
   
      
      
      if (channels[4] > 1600)       // this is ch 5 on the RC   we want it to be off when the radio is off... not so at 1500
      {
         wind_heading = current_boat_heading;  // hold the boat into the wind and hit the manual tack switch while the sails are locked at the start
          gcs_send_text_fmt(PSTR("wind_heading"));
         gcs_send_text_fmt(PSTR("%i"), wind_heading);
          if (wind_heading > 180){desired_boat_heading = wind_heading - 90;}else {desired_boat_heading = wind_heading + 90;}
         gcs_send_text_fmt(PSTR("desired_boat_heading"));
         gcs_send_text_fmt(PSTR("%i"), desired_boat_heading);
         sail_show = 1500;
         wind_heading_set = 1;
         boat = 1; // used to reverse servos on the boat that are different than the servos on my test setup.
      }
      if (wind_heading_set == 1){} else {desired_boat_heading = current_boat_heading;}
   channels[0] = sail_show;   // rear sail up down
   channels[1] = 1000; // lower both sails
   channels[2] = 1500;   
   channels[3] = 1500; // center both sails
   multiwrite(hal.rcout, channels);  //  write to servos   
      return;
   }
    
 /*   
    // this all looks like it needs to be in setup.. need to figure out how ????????????????????????????????????????????????????????????????????????????
    
  // end of start up processing
  //////////////////////////////////////////
  ///////////////////////////////////////////
  
  
  //  x_degrees = 90;
  //  wind_heading = upwind(x_degrees);
   // wind_heading_index = wind_heading/45;
    //if (wind_heading_index < 0 || wind_heading_index > 7){wind_heading_index = 0;}
   // wind_heading_array[wind_heading_index]++; // collect wind heading data
   
   if (pete_gps_speed < 30) // I tried 10 and got nothing
   {
     
     if (level_boat_under_sail_counter == 1)
     {
      level_boat_under_sail  = ahrs.roll_sensor; // / 100; //  adjust for not level boat
      gcs_send_text_fmt(PSTR("level_boat_under_sail")); 
      gcs_send_text_fmt(PSTR("%i"),level_boat_under_sail); 
     } 
     level_boat_under_sail_counter++; level_boat_under_sail_total = level_boat_under_sail_total + level_boat_under_sail;
      if (level_boat_under_sail_counter > 1000){level_boat_under_sail_counter = 0;}
    
   }
  
   //  33.9084656,  33.9094709, -0.00100530000000276 lat for a good sail day boat stayed on north side of laguna lake feb 14 2014
   // it looks like a .001 of a degree is time to tack
    float dt_home_lat = home.lat - current_loc.lat;
    if (dt_home_lat > .0015)    // .001 gave me 400 tack counts
    {
     lat_tack_counter++;
    }
  */ 
  
  
 //      if ( ( heel > 500 ) && (pete_gps_speed > 100) && (heel < 2500)) {tack_for_speed++; tack = 1; gcs_send_text_fmt(PSTR("speed tack"));}
/*   
   range_finder =  rf->read();  /// petes sonar is pointed straight forward , in centameters I think

 if ((range_finder > 100) && (range_finder < 500) && (pete_gps_speed > 30) && (heel > 200) && (heel < 2500) && (pete_dt_home_lat_ft < 600))
 {
   tack = 1;  tack_reason = 66;
   gcs_send_text_fmt(PSTR("range_finder"));
   gcs_send_text_fmt(PSTR("%i"), range_finder);
 }
  
    // tack if we are 30 feet south of launch latitude...  the 600 is so it does not triger when I test at home
 //   if ((pete_dt_home_lat_ft > 30) && (pete_dt_home_lat_ft < 600) && (current_boat_heading > 90) && (current_boat_heading < 270)) {south_limit++;}
 //   if ((current_boat_heading < 90) || (current_boat_heading > 270)) {south_limit = 0;}
    
  
  
     //////////////////////////////////////////////////////////////////////////////
                       /////////////              ////////////////
                       ///////////// sail stuff ////////////////
                       /////////////            ////////////////
       //////////////////////////////////////////////////////////////////////////
    
    
   
    
    
    
    
 //   look for accel... when gravity goes down is a good time to tack
// # I think AccZ < -8 is a swell i.e. freefall data from IMU
// # AccZ > -10 is a trough
 
 
    // need to write code to find height and period of swells... use pitch maybe????????????
                          
    // a way to record the capsize limits of the boat is to look for up/down nose pitch over 60 degrees
   // then just record the highest roll on each side and reset the highest roll to zero
    // or half manual... every time the nose pitch is over 45 , zero the highest roll up to that point
   // then I can see it on the log file
  
    // good random test code     
        // test code for odd conditions
    //    long randNumber;
 //  randNumber = random() % 30;
//   heel = randNumber;
//randomSeed(analogRead(0));
//  randNumber = random() % 360;
  //   desired_boat_heading =  randNumber;  
  //     gcs_send_text_fmt(PSTR("desired_boat_heading, %i"), desired_boat_heading);
  //   randNumber = random() % 360;
   //   current_boat_heading = randNumber;
      //
      
      
      
             // end_time = millis();
               //difference_time = end_time - start_time;

// decide new course based on current wind heading , current speed, and next waypoint   TO DO




*/

// tack every 99 seconds ... a timed tack should wait for a swell so maybe we need a "tack_any_time_now" 
        next_timed_tack = tack_end_time + 220; // every 55 seconds
        if ((pete_time_in_tenth_of_seconds() > next_timed_tack) && (tack == 0)) 
      {
       tack = 1; tack_reason = 99;
      gcs_send_text_fmt(PSTR("time tack")); tack_end_time = tack_end_time + 990; // this way we get only the one message
      }

// range_finder =  rf->read();
/*

int16_t temp_alt = sonar->read();
//if(temp_alt > 200) {
//do_loiter_time();

range_finder = temp_alt;



    previous_range_finder = range_finder;
 range_finder =  water_depth(rf->read());

 
 if (range_finder < 40)
 {
   if (shallow_water == false){deep_to_shallow++; gcs_send_text_fmt(PSTR("deep_to_shallow"));gcs_send_text_fmt(PSTR("%i"), deep_to_shallow);}
   shallow_water= true;
   
 }
 else 
 {
   if (shallow_water == true)
   {
   shallow_to_deep++; 
   //gcs_send_text_fmt(PSTR("shallow_to_deep"));
   //gcs_send_text_fmt(PSTR("%i"), shallow_to_deep);
   }
   shallow_water = false;
 }
 
  */
 
 
 
//int32_t  latitude;
  //  int32_t  longitude;
    
    //int pete_latitude    = current_loc.lat;
      // int pete_longitude   = current_loc.lng;
      
// from log pde
//int pete_ground_speed   = (uint16_t)(ground_speed*100);
//int pete_battery_voltage         = (int16_t)(battery.voltage() * 100.0);
//int        pete_current_amps           = (int16_t)(battery.current_amps() * 100.0);
//   int     pete_board_voltage          = board_voltage();
//      int  pete_current_total          = battery.current_total_mah();
// int pete_get_state = APM_RC.GetState();



/* from
http://diydrones.com/forum/topics/issue-with-analog-input-ports-on-amp-2-6?commentId=705844%3AComment%3A1634726
AP_HAL::AnalogSource* testain;
testain = hal.analogin->channel(0); // read A0 ??
float testain1 = testain->read_average();
cliSerial->printf_P(PSTR("Batery volts = %4.4f \n"),testain1);

/// then I saw this+++++++++++=
private:
    AP_HAL::AnalogSource *_source;
    const AP_Int8 &_pin;
    int8_t _last_pin;
// ++++++++++++++++

AP_HAL::AnalogSource *testain;
testain = hal.analogin->channel(7); // read A7 pete
float testain1 = testain->read_average();
//cliSerial->printf_P(PSTR("Batery volts = %4.4f \n"),testain1);
gcs_send_text_fmt(PSTR("\n Batery volts = %4.4f \n"),testain1);



range_finder =  rf->read(); // now wind vane
 pete_sonar1_distance = (uint16_t)sonar.distance_cm();
        pete_sonar2_distance = (uint16_t)sonar2.distance_cm();
        pete_detected_count  = obstacle.detected_count;
gcs_send_text_fmt(PSTR("petes,%i,%i,%i"),range_finder,pete_sonar1_distance, pete_sonar2_distance,pete_detected_count);
*/

// 33.9085 south lat at Laguna lake
// float pete_latitude    = current_loc.lat; 
//if ((tack_end_time + 200) < pete_time_in_tenth_of_seconds())
//{
//if (pete_latitude < 33.9085 ){ tack = 1;}  // or laguna_lake_south_shore
//}


 float v  = pete_ch->voltage_average(); // wind vane pete
  wind_to_boat_heading = v * 74;   // wind vane pete
 


return;
}   // end  of sailing loop
    ////////////////////////////////////////////////////////////////////
    
 
