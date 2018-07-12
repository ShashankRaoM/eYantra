/*
*Team Id: 717
*Author list:  Ronak, Rohit P, Shashank Rao M, Sangameshwar V
*Filename:  eyrc.c
*Theme: Harvester Bot

*Functions:  buzzer_pin_config(void),port_init(),velocity(unsigned char, unsigned char),motors_delay(),buzzer_pin_config (void),buzzer_on (void),
			 buzzer_off (void),lcd_port_config (void),adc_pin_config (void),left_encoder_pin_config (void),right_encoder_pin_config (void),
			 motion_pin_config (void),port_init(),lcd_port_config(),adc_pin_config(),motion_pin_config(),buzzer_pin_config(),
			 left_position_encoder_interrupt_init (void),right_position_encoder_interrupt_init (void),motion_set (unsigned char),ISR(INT5_vect) ,
			 ISR(INT4_vect),forward2 (void),back2 (void),left2 (void),right2 (void),stop2 (void),angle_rotate(unsigned int),linear_distance_mm(unsigned int),
			 forward_mm(unsigned int),back_mm(unsigned int),left_degrees(unsigned int),right_degrees(unsigned int),timer5_init(),timer1_init(),timer4_init(),
			 adc_init(),ADC_Conversion(unsigned char),print_sensor(char, char,unsigned char),velocity (unsigned char, unsigned char),servo_1(unsigned char),
			 servo_2(unsigned char),servo_3(unsigned char),servo_4(unsigned char),servo_1_free(void),servo_2_free (void),servo_3_free (void),servo_4_free (void),
			 init(),init_devices (void),make_pair1(int,int),make_pair2(float,struct Pair),push(struct Pair),pop(),insert(struct pair_and_heuristic),struct set *erase(void),
 			 find_dep_zone(),isValid(int,int),isUnBlocked(int [][],int,int),isDestination(int,int,struct Pair),heuristic_value_calculator(int, int col, struct Pair destination_node),
			 path_producer(struct cell,struct Pair),Astar_algorithm(int, struct Pair, struct Pair),forward (void),stop (void),right(void),left(void),reverse(void),
			 orient_reverse(void),updirect(int),orientbot(),fruitpluck(void),deposit(),flag_update_if_not_changed(),getnode(void),traverse(void).
			 
*Global Variables: int tree_flag,int fruit_distinguisher,int fruit_recognized_flag,float epsilon,int tree[4],int tree_count,int orange_deposition,int apple_deposition,int blueberry_deposition
				   int pathf[40],int currnode[2],int nextnode[2],int cpoint,int npoint,int direction1,int desti[80],int exit1,int count,int backflag,int fruitconf
				   unsigned char ADC_Value,unsigned char flag,unsigned char Left_sensor_value,unsigned char c,unsigned char Right_sensor_value,int desti[80],struct Node *top,struct set *start,
				   struct set *end.



*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define F_CPU 14745600
#include <math.h> //included to support power function
#include "lcd.c"
#include<stdio.h>
#include<stdlib.h>
#include<float.h>
#include<math.h>
#include<string.h>
#define TREES 3
//#include<stdbool.h>
#define ROW 7
#define COL 7
float epsilon = 0.000000001;
typedef int bool;
enum { false, true };
int i=1;
int tree_flag=0;
int fruit_distinguisher=0;
int fruit_recognized_flag=0;
/////////////////////////////////////////////////////////
//enter tree
int tree[3]={9,29,18};             //insert tree node
int tree_count=0;
//enter deposition zones
int orange_deposition,blueberry_deposition,apple_deposition;

///////////////////////////////////
int blueberry_deposition_zones[4]={35,36,42,43};
int apple_deposition_zones[4]={37,38,44,45};
int orange_deposition_zones[4]={40,41,47,48};
int valid_deposition_node;
int pathf[40];
int currnode[2];
int nextnode[2];
int cpoint=0;
int npoint=0;                                             
int direction1=2;
int desti[80];
int exit1=0;
int count;
int backflag=0;


int fruitconf;
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_sensor_value = 0;
unsigned char Center_sensor_value = 0;
unsigned char Right_sensor_value = 0;

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees;

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;        //Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;        //Setting PORTC 3 logic low to turnoff buzzer
}



void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}



//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{

 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
 DDRD=DDRD | 0xE0;                                                           // For pi and atmega communication
 PORTD=PORTD |0x00;                                                           //Initial condition'
}

//Function to Initialize PORTS
void port_init()
{
    lcd_port_config();
    adc_pin_config();
    motion_pin_config();
    buzzer_pin_config();
    
     motion_pin_config(); //robot motion pins config
     left_encoder_pin_config(); //left encoder pin config
     right_encoder_pin_config(); //right encoder pin config     
    DDRB = DDRB | 0x20; //making PORTB 5 pin output
    PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
    DDRB = DDRB | 0x40; //making PORTB 6 pin output
    PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
    DDRB = DDRB | 0x80; //making PORTB 7 pin output
    PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;         // removing upper nibbel for the protection
 PortARestore = PORTA;         // reading the PORTA original status
 PortARestore &= 0xF0;         // making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore;         // executing the command
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

void forward2 (void) //both wheels forward
{
  motion_set(0x06);
}

void back2 (void) //both wheels backward
{
  motion_set(0x09);
}

void left2 (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right2 (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}



void stop2 (void)
{
  motion_set(0x00);
}




void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop2(); //Stop robot
}

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
      break;
  }
 } 
 stop2(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward2();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back2();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left2(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right2(); //Turn right
 angle_rotate(Degrees);
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
    TCCR5B = 0x00;    //Stop
    TCNT5H = 0xFF;    //Counter higher 8-bit value to which OCR5xH value is compared with
    TCNT5L = 0x01;    //Counter lower 8-bit value to which OCR5xH value is compared with
    OCR5AH = 0x00;    //Output compare register high value for Left Motor
    OCR5AL = 0xFF;    //Output compare register low value for Left Motor
    OCR5BH = 0x00;    //Output compare register high value for Right Motor
    OCR5BL = 0xFF;    //Output compare register low value for Right Motor
    OCR5CH = 0x00;    //Output compare register high value for Motor C1
    OCR5CL = 0xFF;    //Output compare register low value for Motor C1
    TCCR5A = 0xA9;    /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
                       For Overriding normal port functionality to OCRnA outputs.
                        {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
 
    TCCR5B = 0x0B;    //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
 
 
}
void timer1_init()
{
    TCCR1A = 0x00;
 
    ICR1 = 1023; //TOP = 1023
    TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
    TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
    OCR1A = 1023;
    OCR1B = 1023;
    OCR1C = 1023;
    TCCR1A = 0xAB;
    //COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0
    //For Overriding normal port functionality to OCRnA outputs. WGM11=1, WGM10=1. Along With
    //WGM12 in TCCR1B for Selecting FAST PWM Mode

    TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

void timer4_init()
{
     TCCR4B = 0x00; //stop
    TCNT4H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
    TCNT4L = 0x01; //Counter low value to which OCR1xH value is to be compared with
    OCR4AH = 0x03; //Output compare Register high value for servo 1
    OCR4AL = 0xFF; //Output Compare Register low Value For servo 1
    OCR4BH = 0x03; //Output compare Register high value for servo 2
    OCR4BL = 0xFF; //Output Compare Register low Value For servo 2
    OCR4CH = 0x03; //Output compare Register high value for servo 3
    OCR4CL = 0xFF; //Output Compare Register low Value For servo 3
    ICR4H = 0x03;
    ICR4L = 0xFF;
    TCCR4A = 0xAB;
//COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0
//For Overriding normal port functionality to OCRnA outputs. WGM11=1, WGM10=1. Along With //WGM12 in
//    TCCR1B for Selecting FAST PWM Mode TCCR1C = 0x00;
    TCCR4B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
 
}
void adc_init()
{
    ADCSRA = 0x00;
    ADCSRB = 0x00;        //MUX5 = 0
    ADMUX = 0x20;        //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
    ACSR = 0x80;
    ADCSRA = 0x86;        //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
    unsigned char a;
    if(Ch>7)
    {
        ADCSRB = 0x08;
    }
    Ch = Ch & 0x07;           
    ADMUX= 0x20| Ch;            
    ADCSRA = ADCSRA | 0x40;        //Set start conversion bit
    while((ADCSRA&0x10)==0);    //Wait for conversion to complete
    a=ADCH;
    ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
    ADCSRB = 0x00;
    return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
 
    ADC_Value = ADC_Conversion(channel);
    lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
    OCR5AL = (unsigned char)left_motor;
    OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction


//Sets servo 1 to the specified angle in degrees
void servo_1(unsigned char degrees)
{
    float regval = ((float)degrees * 0.512) + 34.56;
    OCR4BH=0x00;
	OCR4BL=(uint16_t) regval;
}

//Sets servo 2 to the specified angle in degrees
void servo_2(unsigned char degrees)
{
    float regval = ((float)degrees * 0.512) + 34.56;
    OCR1B = (uint16_t) regval;
}

//Sets servo 3 to the specified angle in degrees
void servo_3(unsigned char degrees)
{
    float regval = ((float)degrees * 0.512) + 34.56;
    OCR1C = (uint16_t) regval;
}

void servo_4(unsigned char degrees)
{
    float regval = ((float)degrees * 0.512) + 34.56;
    OCR4CH = 0x00;
    OCR4CL = (uint16_t) regval;
}

//Frees (relaxes) servo 1 by sending a continuous on signal
void servo_1_free (void)
{
    OCR4B = 1023;
}

//Frees (relaxes) servo 2 by sending a continuous on signal
void servo_2_free (void)
{
    OCR1B = 1023;
}

//Frees (relaxes) servo 3 by sending a continuous on signal
void servo_3_free (void)
{
    OCR1C = 1023;
}

void servo_4_free (void)
{
    OCR4C = 1023;
}


//Initialises devices
void init()
{
    cli();
    port_init();
    left_position_encoder_interrupt_init();
     right_position_encoder_interrupt_init();
    lcd_port_config();
    timer1_init();
    timer4_init();
    sei();
}

void init_devices (void)
{
     cli(); //Clears the global interrupts
    port_init();
    adc_init();
    timer5_init();
    sei();   //Enables the global interrupts
}



int desti[80];
struct Pair
{
int first;
int second;
};

struct pair_and_heuristic
{
float first;
struct Pair second;
};


/*
Function Name: make_pair1
Input: int,int
Output: struct Pair
Logic: The below function is used to create a Pair to be inserted in stack or set. This function is called to make a Pair out of x and y 
	   coordinates of source and destination.
Example Call: struct pair source_node;
			  struct pair destination_node;
			  source_node=make_pair1(1,1);
			  destination_node=make_pair1(5,4);
			  //Here the Pair source_node and Pair destination_node now contains 1,1 and 5,4 respectively.
*/
			  
struct Pair make_pair1(int a,int b)
    {
      //struct Pair *St = (struct Pair*)malloc(sizeof(struct Pair));
        struct Pair St;
        St.first=a;
        St.second=b;
        return(St);
    }

/*
Function Name: make_pair2
Input: float,struct Pair
Output: struct pair_and_heuristic
Logic: The below function is used to create a pair_and_heuristic instance. This function called so as to insert the elements into set.
Example Call: struct pair_and_heuristic instance_1;
			  struct Pair source_node;
			  source_node=make_pair1(1,1);
			  instance_1= make_pair2(1.8,source_node);
			  //Here instance_1 has heuristics 1.8 and Pair of coordinates 1.1
*/
    
struct pair_and_heuristic make_pair2(float a,struct Pair b)
    {
      //struct pair_and_heuristic *St = (struct pair_and_heuristic*)malloc(sizeof(struct pair_and_heuristic));
        struct pair_and_heuristic St;
        St.first=a;
        St.second.first=b.first;
        St.second.second=b.second;
        return(St);
    }


struct Node
    {
       struct Pair data;
       struct Node *next;
    };
struct Node *top = NULL;


/*
Function Name:push
Input: struct Pair
Output: Void
Logic: The fucntion uses linked list method to create a stack of Pairs where each Pair is linked to the next.
	   The top value of the stack is pointed by the pointer 'top'
Function Call: struct Pair instance_1;
			   push(instance_1);
*/

void push(struct Pair value)
    {
       struct Node *newNode;
       newNode = (struct Node*)malloc(sizeof(struct Node));     
       newNode->data.first = value.first;
       newNode->data.second = value.second;
       if(top == NULL)
          newNode->next = NULL;
       else
          newNode->next = top;
       top = newNode;
       
    }


/*
Function Name:pop
Input: VOid
Output: Void
Logic: This function is used to pop the top element in stack. The top element is first refrenced and the value is obtained and then
	   is made free to pop it.
Function Call: pop();
			   //The function also decrements top pointer and the top pointer now points to the next top element in the stack.
*/

    void pop()
    {
       if(top == NULL)
          printf("\nStack is Empty!!!\n");
       else
       {
          struct Node *temp = top;
          //printf("\nDeleted element: %d,%d\n", temp->data.first,temp->data.second);
          top = temp->next;
          free(temp);
       }
    }
    
    
    struct set
    {
      struct pair_and_heuristic pa;
      struct set* next;
    };

    struct set *start = NULL;
    struct set *end=NULL;
	
	/*
	*Function name: insert()
	*input:  struct pair_and_heuristic
	*output: Void. 
	*Logic:  The fumction is used to create a set of pair_and_heuristic. The set is later referred for finding the path 
	         according to the heuristics.
	*Example Call: insert(struct pair_and_heuristic instance1). The beside command appends the instance1 to the set.
				//The pair_and_heuristic with the lowest heuristics is appended at the first and the one with highest 	
				  heuristic is appended at last.
	*/
	
	
    void insert(struct pair_and_heuristic p)
    {
      
      int flag=1;
      struct set* temp = (struct set*)malloc(sizeof(struct set));
      struct set* temp2 = (struct set*)malloc(sizeof(struct set));
      struct set* temp3 = (struct set*)malloc(sizeof(struct set));
      if(start==NULL && end==NULL)
      {
        //printf("inserted at start\n");
        temp->pa.first=p.first;
        temp->pa.second.first=p.second.first;
        temp->pa.second.second=p.second.second;
        //printf("%lf,(%d,%d) from start\n",temp->pa.first,temp->pa.second.first,temp->pa.second.second);
        start=temp;
        end=temp;
        temp->next=NULL;
      }
      else if(start!=NULL)
      {
        temp=start;
        while(temp!=NULL)
        {
          if(temp->pa.first==p.first)
          {
           if(temp->pa.second.first==p.second.first)
            {
              
              if(temp->pa.second.second==p.second.second)
              {
               //printf("already there\n");
                flag=0;
                break;
                }
                else
                {
                  flag=1;
                }
            }
            else
            {
              flag=1;
            }
          }
          else
          {
          flag=1;
            }
          temp=temp->next;
        }
        
        if(flag!=0)
        {
                    //printf("flag is not zero\n");
                    temp=start;
                    while(temp!=NULL)
                    {
                          if(p.first<=temp->pa.first)
                          { 
                              if(temp==start)
                              {
                                start=temp3;
                                temp3->next=temp;
                                temp3->pa.first=p.first;
                              temp3->pa.second.first=p.second.first;
                              temp3->pa.second.second=p.second.second;
                              break;
                              }
                              //printf("have to append in the middle\n");
                              temp2=start;
                              while(temp2->next!=temp)
                              {
                                  temp2=temp2->next;
                              }
                              temp2->next=temp3;
                              temp3->next=temp;
                              temp3->pa.first=p.first;
                              temp3->pa.second.first=p.second.first;
                              temp3->pa.second.second=p.second.second;
                              break;
                              
                          }
                          temp=temp->next;
                    }
                    if(temp==NULL)
                    {
                      //printf("have to append at the end\n");
                      end->next=temp3;
                      end=temp3;
                      temp3->next=NULL;
                      temp3->pa.first=p.first;
                      temp3->pa.second.first=p.second.first;
                      temp3->pa.second.second=p.second.second;
                    }
        }
        }
        }
	
	/*
	Function Name: erase()
	Input: Void
	Output: Set pointer
	Logic: The below function returns the pointer to the recently popped element from set.
			The element returned has the lowest heuristic value among the other element in the set.
	Example Call: struct set *pa = (struct set*)malloc(sizeof(struct set));
				  pa=erase();
	*/
    struct set *erase(void)
    {
        //printf("ELlement erased\n");
      struct set *pa = (struct set*)malloc(sizeof(struct set));
      pa=start;
      if(pa->next==NULL)
      {
        end=NULL;
        start=NULL;
        return(pa);
      }
      else
      {
        start=start->next;
        return(pa);
      }
    }
    
	/*
	Function Name: find_dep_zone
	Input: Void
	Output: Struct Pair
	Logic: The below functions returns a pair which is the destination coordinates for a specific fruit
	Example Call: struct Pair instance;
				  instance=find_dep_zone()
	*/
	
	
struct Pair find_dep_zone()
{
//input the signal from pi
int orange_x,orange_y,apple_x,apple_y,blueberry_x,blueberry_y;
struct Pair dep_zone;
//if(pin_signal==00)
if(fruit_distinguisher==0)
{
   dep_zone=make_pair1(1000,1000);
    return(dep_zone);
}
else if(fruit_distinguisher==1)
{
  apple_x=apple_deposition/7;
    apple_y=apple_deposition%7;
    dep_zone=make_pair1(apple_x,apple_y);
    return(dep_zone);
}
else if(fruit_distinguisher==2)
{
  blueberry_x=blueberry_deposition/7;
    blueberry_y=blueberry_deposition%7;
    dep_zone=make_pair1(blueberry_x,blueberry_y);
    return(dep_zone);
}
else if(fruit_distinguisher==3)
{
  orange_x=orange_deposition/7;
    orange_y=orange_deposition%7;
    dep_zone=make_pair1(orange_x,orange_y);
    return(dep_zone);
}
return(dep_zone) ;
}

 
 
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    float f, g, h;
};

/*
Function Name: isValid(int,int)
Input: int,int
Output: bool where 1-True and 0-False
Logic: The funtion is used to check if the tree coordinates are valid or not.
	   This function checks whether the y and x coordinates lie in the 0-ROW and 0-COL respectively
Example call: if(isValid(2,3))
					printf("The coordinates are in the range");
*/

bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

/*
Function Name: isUnblocked(grid,int,int)
Input: grid,int,int
Output: bool where 1-True and 0-False
Logic: The funtion is used to check if the coordinates is to be taken as obstacle.
	   This function checks whether the corresponding node contains a tree or an obstacle.
Example call: if(isUnBlocked(grid,2,3))
					printf("The coordinates are in the range");
			//grid here is a 2D array of ROW,COL with elements either 1,0 
			//0 denotes obstacle.
*/

bool isUnBlocked(int grid[][COL], int row, int col)
{
    // Returns true if the cell is not blocked else false
    if (grid[row][col] == 1)
    {
           return (true);
    }
    else
    {
        return (false);
    }
}
 
/*
Function Name: isDestination(int,int,struct Pair)
Input: int,int,struct Pair
Output: bool where 1-True and 0-False
Logic: The function checks whether the int,int is equal to Pair._instance.first, Pair._instance.second
Example call: if(isDestination(2,3, instance_1))
					printf("The considered node is the destination");
*/

bool isDestination(int row, int col,struct Pair destination_node)
{
    if (row == destination_node.first && col == destination_node.second)
        {
          return (true);
        }
    else
    {
        return (false);
    }
}

// A Utility Function to calculate the 'h' heuristics.
float heuristic_value_calculator(int row, int col, struct Pair destination_node)
{
    // Return using the distance formula
    
    return ((float)sqrt ((row-destination_node.first)*(row-destination_node.first)
                          + (col-destination_node.second)*(col-destination_node.second)));
}
    
/*
Function Name:path_producer
Input: struct cell cell_instance,struct pair Pair_instance
Output:void
Logic: The below function is used to produce a shortest path according to least heurisic as the key.
	   The function pushes each cell coordinate into a stack till the destination is reached. The path is pused from destination node to start node 	
	   by accessing the parent cell of each cell under consideration. The stack contains the Pair as its data structure.
	   The stack is then popped to prodeuce pair which is then referenced to find the x,y coordinate.
Example Call: cell_values[2][3].parent_i = 3;
              cell_values[2][3].parent_j = 3;
              printf ("The destination cell is found\n");
              path_producer (cell_values, destination_node);
	   //The path_producer function is called after the isDestination() function returns true
*/

void path_producer(struct cell cell_values[][COL],struct Pair destination_node)
{
    //printf ("\nThe Path is ");
    int row = destination_node.first;
    int col = destination_node.second;
     //printf("came into trace path\n");
    while (!(cell_values[row][col].parent_i == row
             && cell_values[row][col].parent_j == col ))
    {    
            ////printf("came into while\n");
        push (make_pair1 (row, col));
        int temp_row = cell_values[row][col].parent_i;
        int temp_col = cell_values[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
         struct Node *nd = (struct Node*)malloc(sizeof(struct Node));
        nd=top;
        struct Pair *p2 = (struct Pair*)malloc(sizeof(struct Pair));
    push (make_pair1 (row, col));
    for(i=0;i<count;i++)
    {
        pathf[i]=0;
    }
    while (top!=NULL)
    {
            nd=top;
        p2->first=nd->data.first;
        p2->second=nd->data.second;
        pop();
        printf("-> (%d,%d) ",p2->first, p2->second);
        pathf[i]=p2->first;
        i++;
        pathf[i]=p2->second;
        i++;
        count++;
    }
    printf("\n");
    return;
}



// A Star algorithm is used to find the shortest path between source and destination in 
// a obstacle filled grid.
void Astar_algorithm(int grid[][COL], struct Pair source_node, struct Pair destination_node)
{    
	//Reset set pointers and stack pointers to null to start new operation
      top=NULL;         
      start=NULL;
      end=NULL;
      cpoint=0;
      count=0;
      backflag=0;
      exit1=0;
    //Return if the source is invalid
    if (isValid (source_node.first, source_node.second) == false)
    {
        
        //printf ("Source is invalid\n");
        return;
    }
 
    // Return if the destination is invalid
    if (isValid (destination_node.first, destination_node.second) == false)
    {
        //printf ("Destination is invalid\n");
        return;
    }
 
    // Return if the destination or source is blocked
    if (isUnBlocked(grid, source_node.first, source_node.second) == false ||
            isUnBlocked(grid, destination_node.first, destination_node.second) == false)
    {
        printf ("Source or the destination is blocked\n");
        return;
    }
 
    // Return if both source and destination are same
    if (isDestination(source_node.first, source_node.second, destination_node) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }
 
	// Creating a closed list to initialize all the elements to False as they aren't visited yet.
	//This is a boolean 2D array where the elements are updated to True once the cell is included
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));
    //memset is used to store all the elements of the array to certain value...Here it is False(0)
    //Creating a cell instance for astar search
	struct cell cell_values[ROW][COL];
     
    int i, j;
 
    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            
            cell_values[i][j].f = FLT_MAX;
			//FLT_MAX stores the maximum value that float can hold
            cell_values[i][j].g = FLT_MAX;
			
            cell_values[i][j].h = FLT_MAX;
			//set the parent values of the cell to -1 as its not visited yet
			//both axes must be considered
            cell_values[i][j].parent_i = -1;
			
            cell_values[i][j].parent_j = -1;
        }
    }
     
    // Initializing the source node to i,j
    i = source_node.first, j = source_node.second;
	//set the distance value of both axes to 0
    cell_values[i][j].f = 0.0;
	
    cell_values[i][j].g = 0.0;
	//heuristic h is the distance
    cell_values[i][j].h = 0.0;
	
    cell_values[i][j].parent_i = i;
	
    cell_values[i][j].parent_j = j;
    //Create a pair_and_heuristic instance with the current node coordinates and heuristic value
	//insert the above instance to set
	//Creating the openlist
    insert(make_pair2 (0.0, make_pair1 (i, j)));
    
    //initialize the founddest flag to 0 as the destination is not yet reached
    bool foundDest = false;
    
    
    //start of set erasing functions.
	//the set pointer points to the pair_and _heuristic instance with least heuristics.
	//Node with least heuristics is the one which is closest to destination
    struct set *p = (struct set*)malloc(sizeof(struct set));
    while (start!=NULL)
    {
      
        // Remove the vertex from the open list
        p=erase();
 
        // Add this vertex to the open list
        i = p->pa.second.first;
        j = p->pa.second.second;
        //make the node with coordinate(i,j) as included in closed list.
        closedList[i][j] = true;
      
        //Create new variables to store updated distance values     
        float Updated_g_cost_function, Updated_h_cost_function, Updated_f_cost_function;
        
        
        //For a node there a 4 possible direction which can be chosen in the worst case scenario.
		
		//NORTH NODE 
 
        //Check if the north node is a valid node
		//If not valid jump the process and search other nodes
        if (isValid(i-1, j) == true)
        {
            //check if the cell is the destination
			//If the node is the destination, then print the path else update cost variables.
            if (isDestination(i-1, j, destination_node) == true)
            {
                // Set the Parent of the destination cell
                cell_values[i-1][j].parent_i = i;
				
                cell_values[i-1][j].parent_j = j;
				
                printf ("The destination cell is found\n");
				
                path_producer (cell_values, destination_node);//printing the path
				
                foundDest = true;
				
                return;
            }
			
			//Ignore if the node is blocked or already included
            else if (closedList[i-1][j] == false &&
                     isUnBlocked(grid, i-1, j) == true)
            {
				// Set the Parent of the destination cell
                Updated_g_cost_function = cell_values[i][j].g + 1.0;
				  
                Updated_h_cost_function = heuristic_value_calculator (i-1, j, destination_node);
				
				Updated_f_cost_function = Updated_g_cost_function + Updated_h_cost_function;
 
				//add the cell to openlist if not added. Make the current node the parent node of the next considered node.
				//If the 'f' cost function is better,then consider the current node.
				
                if (cell_values[i-1][j].f == FLT_MAX ||
                        cell_values[i-1][j].f > Updated_f_cost_function)
                {
					//Insert the current cell to the set
                    insert( make_pair2(Updated_f_cost_function,make_pair1(i-1, j)));
 
                    // Update the details of this cell
                    cell_values[i-1][j].f = Updated_f_cost_function;
					
                    cell_values[i-1][j].g = Updated_g_cost_function;
					
                    cell_values[i-1][j].h = Updated_h_cost_function;
					
                    cell_values[i-1][j].parent_i = i;
					
                    cell_values[i-1][j].parent_j = j;
                }
            }
        }
 
        //SOUTH NODE 
 
        //Check if the north node is a valid node
		//If not valid jump the process and search other nodes
        if (isValid(i+1, j) == true)
        {
            
            //check if the cell is the destination
			//If the node is the destination, then print the path else update cost variables.
            if (isDestination(i+1, j, destination_node) == true)
            {
                
                // Set the Parent of the destination cell
                cell_values[i+1][j].parent_i = i;
				
                cell_values[i+1][j].parent_j = j;
				
                printf("The destination cell is found\n");
				
                path_producer(cell_values, destination_node);
				
                foundDest = true;
				
                return;
            }
			
            //Ignore if the node is blocked or already included
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(grid, i+1, j) == true)
            {
                // Set the Parent of the destination cell
                Updated_g_cost_function = cell_values[i][j].g + 1.0;
				
                Updated_h_cost_function = heuristic_value_calculator(i+1, j, destination_node);
				
				Updated_f_cost_function = Updated_g_cost_function + Updated_h_cost_function;
 
                //add the cell to openlist if not added. Make the current node the parent node of the next considered node.
				//If the 'f' cost function is better,then consider the current node.
                if (cell_values[i+1][j].f == FLT_MAX ||
                        cell_values[i+1][j].f > Updated_f_cost_function)
                {
                    //Insert the current cell to the set
                    insert( make_pair2(Updated_f_cost_function, make_pair1 (i+1, j)));
					
                    // Update the details of this cell
                    cell_values[i+1][j].f = Updated_f_cost_function;
					
                    cell_values[i+1][j].g = Updated_g_cost_function;
					
                    cell_values[i+1][j].h = Updated_h_cost_function;
					
                    cell_values[i+1][j].parent_i = i;
					
                    cell_values[i+1][j].parent_j = j;
                }
            }
        }
 
        //EAST NODE............
 
        //Check if the north node is a valid node
		//If not valid jump the process and search other nodes
        if (isValid (i, j+1) == true)
        {
            
            //check if the cell is the destination
			//If the node is the destination, then print the path else update cost variables.
            if (isDestination(i, j+1, destination_node) == true)
            {
                
                // Set the Parent of the destination cell
                cell_values[i][j+1].parent_i = i;
				
                cell_values[i][j+1].parent_j = j;
				
                printf("The destination cell is found\n");
				
                path_producer(cell_values, destination_node);
				
                foundDest = true;
				
                return;
            }
 
            //Ignore if the node is blocked or already included
            else if (closedList[i][j+1] == false &&
                     isUnBlocked (grid, i, j+1) == true)
            {
                // Set the Parent of the destination cell
                Updated_g_cost_function = cell_values[i][j].g + 1.0;
				
                Updated_h_cost_function = heuristic_value_calculator (i, j+1, destination_node);
				
				Updated_f_cost_function = Updated_g_cost_function + Updated_h_cost_function;
				
				if (cell_values[i][j+1].f == FLT_MAX ||
                        cell_values[i][j+1].f > Updated_f_cost_function)
                {
                    
                    insert( make_pair2(Updated_f_cost_function,
                                        make_pair1(i, j+1)));
 
                    
                    cell_values[i][j+1].f = Updated_f_cost_function;
					
                    cell_values[i][j+1].g = Updated_g_cost_function;
					
                    cell_values[i][j+1].h = Updated_h_cost_function;
					
                    cell_values[i][j+1].parent_i = i;
					
                    cell_values[i][j+1].parent_j = j;
                }
            }
        }
 
        //WEST node
 
        if (isValid(i, j-1) == true)
        {
            
            if (isDestination(i, j-1, destination_node) == true)
            {
                
               
                cell_values[i][j-1].parent_i = i;
				
                cell_values[i][j-1].parent_j = j;
				
                printf("The destination cell is found\n");
				
                path_producer(cell_values, destination_node);
				
                foundDest = true;
				
                return;
            }
 
            
            else if (closedList[i][j-1] == false &&
                     isUnBlocked(grid, i, j-1) == true)
            {
                
                Updated_g_cost_function = cell_values[i][j].g + 1.0;
				
                Updated_h_cost_function = heuristic_value_calculator(i, j-1, destination_node);
				
                Updated_f_cost_function = Updated_g_cost_function + Updated_h_cost_function;
				
 
                if (cell_values[i][j-1].f == FLT_MAX ||
                        cell_values[i][j-1].f > Updated_f_cost_function)
                {
                    
                    insert( make_pair2(Updated_f_cost_function,
                                          make_pair1(i, j-1)));
 
            
                    cell_values[i][j-1].f = Updated_f_cost_function;
					
                    cell_values[i][j-1].g = Updated_g_cost_function;
					
                    cell_values[i][j-1].h = Updated_h_cost_function;
					
                    cell_values[i][j-1].parent_i = i;
					
                    cell_values[i][j-1].parent_j = j;
                }
            }
        }
    }
 
    if (foundDest == false)
    {
        printf("Failed to find the Destination Cell\n");
 
    return;
    }
}

void forward (void)
{
  motion_set (0x06);
 return;
}

void stop (void)
{
  motion_set (0x00);
   return;
}
//Used by the bot to take a right turn while traversing and orienting and plucking a fruit
void right(void)
{
    stop();
    _delay_ms(100);
    velocity(200,200);
    forward_mm(60);
    right_degrees(80);
    motion_set(0x0A);
    velocity(120,120);
    while (!(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value<0x0D))
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
    } 
  stop();
  _delay_ms(200);
  return;
}

//Used by the bot to take a left turn while traversing and orienting for plucking a fruit
void left(void)
{
    stop();
    _delay_ms(100);
    velocity(200,200);
    forward_mm(60);
    left_degrees(80);
    motion_set(0x05);
    velocity(120,120);    
  while (!(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value<0x0D))
  {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
        
  }  
  stop();
  _delay_ms(200);
  return;
}

//Used by the bot to take a reverse turn while traversing along the grid
void reverse(void)
{
    stop();
    motion_set(0x0A);
    velocity(150,150);
    _delay_ms(2000);
    while (!(Center_sensor_value<0x0D && Left_sensor_value<0x0D && Right_sensor_value>0x0D))
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value= ADC_Conversion(1);    //Getting data of Right WL Sensor
        
    }  
  stop();
  _delay_ms(200);
  return;
}

//Used by the bot to orient reverse while plucking a fruit
void orient_reverse(void)
{

    stop();
    velocity(185,185);
    forward_mm(70);
    right_degrees(170);
	motion_set(0x0A);
    while (!(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value<0x0D))
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
    }
	stop();
	_delay_ms(300);
    motion_set(0x09);
    velocity(85,85);
    while(1)
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor

        if((Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value<0x0D)||(Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value>0x0D)||(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value>0x0D))
        {
            backflag=1;
            forward();
            _delay_ms(200);
            break;
        }
    }  
stop();
motion_set(0x0a);
velocity(100,100);
_delay_ms(50);
stop();
}

/*
Function Name: updirect
Input: int
Output: void
Logic: This updates the direction of the bot based on the truns the bot makes, the direction of the bot is updated
Example call:updirect(1);
			 updirect(2);
			 updirect(3);

*/
void updirect(int dir)                                                    
{
    switch(dir)
    {
        case 0: if(direction1==0)
                {
                    direction1= 3;
                }
                else
                {
                direction1=direction1-1;
                }
                break;
        case 2: if(direction1==3)
                {
                    direction1=0;
                }
                else
                {
                direction1=direction1+1;
                }
                break;
        case 3: if (direction1==3)
                {
                    direction1=1;
                }
                else if (direction1==2)
                {
                    direction1=0;
                }
                else
                {
                    direction1=direction1+2;
                }
    }
}
/*
Function Name: orientbot()
Input: void
Output: void
Logic: Based on the current node and the tree node, the bot orients in such a way that the arm of the bot
	   is facing the tree at all times
Example call: orientbot();
*/
void orientbot()
 {
    int trow,tcol;
    trow = tree[tree_count]/7;
    tcol = tree[tree_count]%7;
    if(currnode[0]==trow)
    {
            
        if(currnode[1]<tcol)
        {
            if(direction1==1)
            {
                right();
                updirect(0);
                motion_set(0x09);
                velocity(185,185);
                back_mm(60);
                stop();
                return;
            }
            else if(direction1==2)
            {
                orient_reverse();
                updirect(3);
                return;
            }
            else if(direction1==3)
            {
                left();
                updirect(2);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
        }
        if(currnode[1]>tcol)
        {
            if (direction1==0)
            {
                orient_reverse();
                updirect(3);
                return;
            }
            else if(direction1==1)
            {
                left();
                updirect(2);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
            else if(direction1==3)
            {
                right();
                updirect(0);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
        }
    }
    if(currnode[1]==tcol)
    {
        if(currnode[0]<trow)
        {
            if (direction1==0)
            {
                left();
                updirect(2);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
            else if(direction1==2)
            {
                right();
                updirect(0);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
             else if(direction1==3)
            {
                orient_reverse();
                   updirect(3);
                return;
            }
        }
        if(currnode[0]>trow)
        {
            if (direction1==0)
            {
                right();
                updirect(0);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
            else if(direction1==1)
            {
                reverse();
                updirect(3);
                motion_set(0x09);
               velocity(185,185);
                back_mm(25);
                return;
            }
            else if(direction1==2)
            {
                left();
                updirect(2);
                motion_set(0x09);
               velocity(185,185);
                back_mm(60);
                return;
            }
        }
    }
    return;
} 

/*
Function Name: dep_assign();
Input: int
Output: void
Logic : This function assigns the deposition for different fruit deposition zones
Example call: dep_assign(grid);
*/
void dep_assign(int grid[][COL])
{

for(i=0;i<3;i++)
{
    if(isUnBlocked(grid,apple_deposition_zones[i]/7,apple_deposition_zones[i]%7))
    {
        apple_deposition=apple_deposition_zones[i];
        break;
    }
}


for(i=0;i<3;i++)
{
    if(isUnBlocked(grid,blueberry_deposition_zones[i]/7,blueberry_deposition_zones[i]%7))
    {
        blueberry_deposition=blueberry_deposition_zones[i];
        break;
    }
}


for(i=0;i<3;i++)
{
    if(isUnBlocked(grid,orange_deposition_zones[i]/7,orange_deposition_zones[i]%7))
    {
        orange_deposition=orange_deposition_zones[i];
        break;
    }
}


}


/*
Function Name: deposition_zone_allign
Input: int,int
Output: int
Logic: Used to assign the last node for traversal to the node
Example call: value=deposition_zone_assign(fruit_distinguisher,grid);
*/
int deposition_zone_allign(int deposition_node_number,int grid[][COL])
{
 
    int i;
    if(deposition_node_number==1)
    {
     
        for(i=0;i<3;i++)
        {
            if(isUnBlocked(grid,apple_deposition_zones[i]/7,apple_deposition_zones[i]%7))
            {
                return(i);
            }
        }
    }
    else if(deposition_node_number==2)
    {
        for(i=0;i<3;i++)
        {
            if(isUnBlocked(grid,blueberry_deposition_zones[i]/7,blueberry_deposition_zones[i]%7))
            {
                return(i);
            }
        }
    }
   else if(deposition_node_number==3)
   {
      for(i=0;i<3;i++)
        {
            if(isUnBlocked(grid,orange_deposition_zones[i]/7,orange_deposition_zones[i]%7))
            {
                return(i);
            }
        }
   }
 
   return 0;
}
/*
Funciton Name: deposit();
Input: int
Output: void
Logic: This function is used to deposit all the fruits placed in the crate in the respective deposition nodes
Example call: deposit(deposition_node_number);
*/

void deposit(int d)
{
    tree_count++;
    if(d==0)
    {
        if(direction1==0)
        {
            forward_mm(40);
            right_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
           left_degrees(125);
            motion_set(0x05);
        }
        else if(direction1==1)
        {
            forward_mm(40);
            left_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(125);
            motion_set(0x0A);
        }
       
        else if(direction1==2)
        {
            forward_mm(40);
            left_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(35);
            motion_set(0x0A);
         }
        else if(direction1==3)
        {
            forward_mm(40);
            right_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            left_degrees(35);
            motion_set(0x05);
      }
       
    }
   
    else if(d==1)
    {
        if(direction1==0)
        {
            forward_mm(40);
            right_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            left_degrees(35);
            motion_set(0x05);
        }
        else if(direction1==1)
        {
            forward_mm(40);
            right_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            left_degrees(125);
            motion_set(0x05);
        }
       
        else if(direction1==2)
        {
            forward_mm(40);
            left_degrees(135);
            _delay_ms(500);
             servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(125);
            motion_set(0x05);
         }
        else if(direction1==3)
        {
            forward_mm(40);
            left_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
             right_degrees(35);
            motion_set(0x0A);
         }
    }
       
       
    else if(d==3)
    {
        if(direction1==0)
        {
            forward_mm(40);
            left_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(35);
            motion_set(0x0A);    
        }
        else if(direction1==1)
        {
            forward_mm(40);
            right_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
             left_degrees(35);
            motion_set(0x05);
        }
       
        else if(direction1==2)
        {
            forward_mm(40);
            right_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
             left_degrees(125);
            motion_set(0x05);
         }
        else if(direction1==3)
        {
            forward_mm(40);
            left_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(125);
            motion_set(0x0A);
        }
    }
       
    else if(d==2)
    {
        if(direction1==0)
        {
            forward_mm(40);
            left_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            right_degrees(125);
            motion_set(0x0A);       
        }
        else if(direction1==1)
        {
            forward_mm(40);
            left_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500); 
            left_degrees(35);
            motion_set(0x05);
        }
       
        else if(direction1==2)
        {
            forward_mm(40);
            right_degrees(45);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
            left_degrees(35);
            motion_set(0x05);
         }
        else if(direction1==3)
        {
            forward_mm(40);
            right_degrees(135);
            _delay_ms(500);
            servo_4(110);
            _delay_ms(500);
            servo_4(0);
            _delay_ms(500);
             left_degrees(125);
            motion_set(0x05);
         }
    }
       
    velocity(120,120);
    while (!(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value<0x0D))
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
    }
    return;
}

/*
Function Name:flag_update_if_not_changed
Input: void
Output: void
Logic: This function is used to update the tree_distinguisher_flag. If the previous value for the flag is 0,
	   then it changes if the value is not 0 and then locks if it changes.
Example call: flag_update_if_not_changed() 
*/

 void flag_update_if_not_changed()                     
{
	if(fruit_recognized_flag!=1)
	{
		fruit_distinguisher=tree_flag;
		if(fruit_distinguisher!=0)
		{
			fruit_recognized_flag=1;
		}
	}
}
/*
Function call:fruitpluck()
Input: void
Output: void
Logic: This function is used by the bot to orient by calling orientbot() and then scan for the fruit.If an input 
	   is detected at a certain pin, connected to the raspberry pi, then it will pluck the fruit.
Example call: fruitpluck();
*/
 void fruitpluck(void)                                                                               
{
    orientbot();
    int pi_dis=0;
    float i,t;
    lcd_print(2,1,currnode[0],1);
    lcd_print(2,2,currnode[1],1);
    lcd_print(2,3,direction1,1);
    servo_1(35);
    servo_2(210);
    servo_3(50);//open hand
    _delay_ms(500);
	servo_1_free();
    PORTD=  0x20; //pi enable     
	for(i=210;i>175;i=i-1)
    {
        servo_2(i);
        t=i;
        _delay_ms(150);
        pi_dis=PIND & 0x10; // take pi_disable input;
        if(pi_dis)
        {
            tree_flag= PIND & 0x03;
            PORTD =  0x00;
            flag_update_if_not_changed();
			for(i=20;i>=5;i--)
			{
				servo_1(i);
				_delay_ms(50);
			}
			servo_2(195);
			_delay_ms(500);
			for(i=50;i<=100;i++)
			{
				servo_3(i);
				_delay_ms(50);
			}
			servo_2_free();
			for(i=5;i<=35;i++)
			{
				servo_1(i);
				_delay_ms(20);
			}

			servo_1_free();

			for(i=200;i>=15;i--)
			{
				_delay_ms(10);
				servo_2(i);
			}

			_delay_ms(100);
			for(i=35;i>=0;i--)
			{
				servo_1(i);
				_delay_ms(30);
			}
			for(i=100;i>=50;i--)
			{
				servo_3(i);
			}
			_delay_ms(100);
			for(i=0;i<=35;i++)
			{
				servo_1(i);
				_delay_ms(30);
			}
			for(i=15;i<=195;i++)
			{
				servo_2(i);
				_delay_ms(20);
			}
            _delay_ms(500);
			servo_1_free();
          	servo_3_free();
        }
    }  
    tree_flag= PIND & 0x03;
    PORTD =  0x00;
    flag_update_if_not_changed();
    return;
}

/*
Function Name:getnode();
Input:int
Output: void
Logic: Updates the next node that has to be traversed by the bot.
Example call: getnode();
*/
void getnode(int grid[][COL])
{
    currnode[0]=pathf[cpoint];
    cpoint++;
    currnode[1]=pathf[cpoint];
    cpoint++;
    if(cpoint<count*2)
    {
    npoint=cpoint;
    nextnode[0]=pathf[npoint];
    npoint++;
    nextnode[1]=pathf[npoint];
    npoint++;
    }
    else                                    //final destination reached
    {
        forward();                 
        if((cpoint==count*2)&&(fruitconf==0) )                                                                  
        {
            stop();
            _delay_ms(500);
            valid_deposition_node=deposition_zone_allign(fruit_distinguisher,grid);
            deposit(valid_deposition_node);
            _delay_ms(500);
            exit1=1;
			fruit_recognized_flag=0;
            return;
        }
        else if(fruitconf==1)
        {
            stop();
            _delay_ms(500);
            fruitpluck();
            _delay_ms(500);
            exit1=1;
            return;
        }
    }
}
/*
Function Name: traverse();
Input: void
Output: void
Logic: This function deals with the traversing of the bot along the grid in the specified part.
Example Call: traverse();
*/
void traverse(int grid[][COL])
{
    init_devices();
    lcd_set_4bit();
    lcd_init();
    init();
    while(1)
    {
        Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
        Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
        Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
        if(backflag==0)
        { 
            motion_set(0x09);
            velocity(85,85);
            while(1)
            {
                Left_sensor_value = ADC_Conversion(3);    //Getting data of Left WL Sensor
                Center_sensor_value = ADC_Conversion(2);    //Getting data of Center WL Sensor
                Right_sensor_value = ADC_Conversion(1);    //Getting data of Right WL Sensor
   
                if((Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value<0x0D)||(Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value>0x0D)||(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value>0x0D))
                {
                    backflag=1;
                    forward();
                    _delay_ms(200);
                    break;
                }
            }    
        }
        lcd_print(2,1,currnode[0],1);
        lcd_print(2,2,currnode[1],1);
        lcd_print(2,3,direction1,1);  
        flag=0;
        if(Center_sensor_value>0x0D)
        {
            flag=1;
            forward();
            velocity(225,225);
        }
        if((Left_sensor_value<0x0D) && (flag==0))
        {
            flag=1;
            forward();
            velocity(180,90);
        }
        if((Right_sensor_value<0x0D) && (flag==0))
        {
            flag=1;
            forward();
            velocity(90,180);
        }
        if(Center_sensor_value<0x0D && Left_sensor_value<0x0D && Right_sensor_value<0x0D)
        {
            forward();
            velocity(190,190);

        }
        if((Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value<0x0D)||(Center_sensor_value>0x0D && Left_sensor_value>0x0D && Right_sensor_value>0x0D)||(Center_sensor_value>0x0D && Left_sensor_value<0x0D && Right_sensor_value>0x0D))
        {            
            getnode(grid);         
            if(exit1==1)
                return;
            if(currnode[0]==nextnode[0])
            {
                if(currnode[1]<nextnode[1])
                {
                    if (direction1==0)
                    {  
                        left();
                        updirect(2);
                    }
                    else if(direction1==1)
                    {
                        forward();
                        _delay_ms(300);
                    }
                    else if(direction1==2)
                    {
                        
                        right();
                       updirect(0);
                    }
                    else if(direction1==3)
                    {
                        reverse();
                       updirect(3);
                    }
                }
                if(currnode[1]>nextnode[1])
                {
                    if (direction1==0)
                    {
                        right();
                        updirect(0);
                    }
                    else if(direction1==1)
                    {
                        reverse();
                        updirect(3);
                    }
                    else if(direction1==2)
                    {
                        left();
                        updirect(2);
                    }
                    else if(direction1==3)
                    {
                        forward();
                         _delay_ms(300);
                    }
                }
            }
            if(currnode[1]==nextnode[1])
            {
                if(currnode[0]<nextnode[0])
                {
                    if (direction1==0)
                    {
                        reverse();
                        updirect(3);
                    }
                    else if(direction1==1)
                    {
                        left();
                        updirect(2);
                    }
                    else if(direction1==2)
                    {
                        forward();
                        _delay_ms(300);
                    }
                    else if(direction1==3)
                    {
                        right();
                        updirect(0);
                    }
                }
                if(currnode[0]>nextnode[0])
                {
                    if (direction1==0)
                    {
                        forward();
                        _delay_ms(300);
                    }
                    else if(direction1==1)
                    {
                        right();
                        updirect(0);
                    }
                    else if(direction1==2)
                    {
                        reverse();
                        updirect(3);
                    }
                    else if(direction1==3)
                    {
                        left();
                        updirect(2);
                    }
                }
            }
        }
    }
}
int main()
{
	init_devices();
    lcd_set_4bit();
    lcd_init();
    init();
    struct Pair source_node;                    //source pair
    struct Pair destination_node;               //destination pair
    /*
     1--> The Node is not blocked
     0--> The Node is blocked    */
    int grid[ROW][COL] =
    {
        { 1, 1, 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1, 1, 1 }, 
        { 1, 1, 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1, 1, 1 },
        { 1, 1, 1, 1, 1, 1, 1 }
        
    };
    servo_1(35);
    servo_2(210);
	servo_3(50);
	servo_4(0);
    
    int x,y,i;            // The below procedure is to place the trees in the map and make them recognized as obstacle
    for(i=0;i<TREES;i++)
    {
      x=tree[i]/7;
      y=tree[i]%7;
      if(isValid(x,y))
      {
          grid[x][y]=0;                //obstacle if the node is valid
      }
    }
    dep_assign(grid);
   
    
    int c=0;                                //indexing variables
    for(i=0;i<TREES;i++)
    {
      x=tree[i]/7;                             //checking all the adjacent nodes of (x,y)
      y=tree[i]%7;
      if(isValid(x-1,y) && isUnBlocked(grid,x-1,y))
      {
          printf("%d,%d is valid\n",x-1,y);                // the south node
          desti[c]=x-1;                 //Destination array is the array of max length 80 that take x,y values of valid path continuously
          c++;                          //c is the indexing variable used to have a count of value till desti reaches
          desti[c]=y;                  //eg: desti=[1,2,3,4,5,6],,then c is at 5 ie desti[5] is last element
          c++;
      }
      else
          printf("%d,%d is invalid\n",x-1,y );
                                                              //The avove procedudes is called to check all adjacent node
      if(isValid(x,y+1) && isUnBlocked(grid,x,y+1))          //left node
      {
          printf("%d,%d is valid\n",x,y+1);
          desti[c]=x;
          c++;
          desti[c]=y+1;
          c++;
      }
      else
          printf("%d,%d is invalid\n",x,y+1);
          
          
      if(isValid(x+1,y) && isUnBlocked(grid,x+1,y))              //north node
       {
          printf("%d,%d is valid\n",x+1,y);
          desti[c]=x+1;
          c++;
          desti[c]=y;
          c++;
      }
      else
        printf("%d,%d is invalid\n",x+1,y);                   
          
          
      if(isValid(x,y-1) && isUnBlocked(grid,x,y-1))                   //right node
       {
          printf("%d,%d is valid\n",x,y-1);
          desti[c]=x;
          c++;
          desti[c]=y-1;
          c++;
      }
      else
          printf("%d,%d is invalid\n",x,y-1 );
        
    desti[c]=1000;                //Default values for recognition of destination zones
    c++;
    desti[c]=1000;
    c++;
    }
    
    int count;
    count=c;
    source_node = make_pair1(0, 0);
    
    for(i=0;i<count-1;i+=2)
    {
      if(desti[i]==1000 && desti[i+1]==1000)
      {
         destination_node=find_dep_zone(tree_flag);
        fruitconf=0;
      }
      else
      {
            destination_node= make_pair1(desti[i],desti[i+1]);
            fruitconf=1;
      }
      
      if(!isValid(destination_node.first,destination_node.second))
          continue;
      
       Astar_algorithm(grid,source_node,destination_node);
       traverse(grid);
       source_node=make_pair1(destination_node.first,destination_node.second);
    }

	buzzer_on();                //End of Task
	_delay_ms(5000);
	buzzer_off();
    return(0);
}

