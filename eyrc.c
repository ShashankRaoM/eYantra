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
int zoom=0;
int fruit_recognized_flag=0;
/////////////////////////////////////////////////////////
//enter tree
int tree[3]={16,33,26};             //insert tree node
int tree_count=0;
//enter deposition zones
int orange_deposition,blueberry_deposition,apple_deposition;

///////////////////////////////////
int blueberry_deposition_zones[4]={35,36,42,43};
int apple_deposition_zones[4]={39,40,46,47};
int orange_deposition_zones[4]={36,37,43,44};
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
unsigned char L = 0;
unsigned char C = 0;
unsigned char R = 0;

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
 DDRD=DDRD | 0x60;                                                           // For pi and atmega communication
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
	 DDRH=DDRH|0x30;
	 PORTH=PORTH|0x30;
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

 Direction &= 0x0F;         // removing upper nibble for the protection
 PortARestore = PORTA;         // reading the PORTA original status
 PortARestore &= 0xF0;         // making lower direction nibble to 0
 PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
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
	OCR4CH=0x00;
    OCR4CL = (uint16_t) regval;
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
    OCR4BH =0x00;
	OCR4BL = (uint16_t) regval;
}

//Frees (relaxes) servo 1 by sending a continuous on signal
void servo_1_free (void)
{
    OCR4C = 1023;
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
    OCR4B = 1023;
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
	timer4_init();
    sei();   //Enables the global interrupts
}



int desti[80];
struct Pair
{
int first;
int second;
};

struct pPair
{
float first;
struct Pair second;
};

struct Pair make_pair1(int a,int b)
    {
      //struct Pair *St = (struct Pair*)malloc(sizeof(struct Pair));
        struct Pair St;
        St.first=a;
        St.second=b;
        return(St);
    }
  
struct pPair make_pair2(float a,struct Pair b)
    {
      //struct pPair *St = (struct pPair*)malloc(sizeof(struct pPair));
        struct pPair St;
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
      struct pPair pa;
      struct set* next;
    };

    struct set *start = NULL;
    struct set *end=NULL;

    void insert(struct pPair p)
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
  
struct Pair find_dep_zone()
{
//input the signal from pi
int orange_x,orange_y,apple_x,apple_y,blueberry_x,blueberry_y;
struct Pair dep_zone;
//if(pin_signal==00)
if(zoom==0)
{
   dep_zone=make_pair1(1000,1000);
    return(dep_zone);
}
else if(zoom==1)
{
  apple_x=apple_deposition/7;
    apple_y=apple_deposition%7;
    dep_zone=make_pair1(apple_x,apple_y);
    return(dep_zone);
}
else if(zoom==2)
{
  blueberry_x=blueberry_deposition/7;
    blueberry_y=blueberry_deposition%7;
    dep_zone=make_pair1(blueberry_x,blueberry_y);
    return(dep_zone);
}
else if(zoom==3)
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

bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

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
 
bool isDestination(int row, int col,struct Pair dest)
{
    if (row == dest.first && col == dest.second)
        {
          return (true);
        }
    else
    {
        return (false);
    }
}

// A Utility Function to calculate the 'h' heuristics.
float calculateHValue(int row, int col, struct Pair dest)
{
    // Return using the distance formula
  
    return ((float)sqrt ((row-dest.first)*(row-dest.first)
                          + (col-dest.second)*(col-dest.second)));
}
  
void tracePath(struct cell cellDetails[][COL],struct Pair dest)
{
    //printf ("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;
     //printf("came into trace path\n");
    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {  
            ////printf("came into while\n");
        push (make_pair1 (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
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



// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(int grid[][COL], struct Pair src, struct Pair dest)
{  
      top=NULL;
      start=NULL;
      end=NULL;
      cpoint=0;
      count=0;
      backflag=0;
      exit1=0;
       // If the source is out of range
    if (isValid (src.first, src.second) == false)
    {
      
        //printf ("Source is invalid\n");
        return;
    }
 
    // If the destination is out of range
    if (isValid (dest.first, dest.second) == false)
    {
        //printf ("Destination is invalid\n");
        return;
    }
 
    // Either the source or the destination is blocked
    if (isUnBlocked(grid, src.first, src.second) == false ||
            isUnBlocked(grid, dest.first, dest.second) == false)
    {
        printf ("Source or the destination is blocked\n");
        return;
    }
 
    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }
 
    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));
       // Declare a 2D array of structure to hold the details
    //of that cell
    struct cell cellDetails[ROW][COL];
   
    int i, j;
 
    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
          
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }
   
    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;
  
    insert(make_pair2 (0.0, make_pair1 (i, j)));
  
  
    bool foundDest = false;
  
  
  
    struct set *p = (struct set*)malloc(sizeof(struct set));
    while (start!=NULL)
    {
    
        //pPair p = *openList.begin();
            // Remove this vertex from the open list
        p=erase();
 
        // Add this vertex to the open list
        i = p->pa.second.first;
        j = p->pa.second.second;
      
        closedList[i][j] = true;
    
       /*
        Generating all the 8 successor of this cell
 
            N.W   N   N.E
              \   |   /
               \  |  /
            W----Cell----E
                 / | \
               /   |  \
            S.W    S   S.E*/
          
        float gNew, hNew, fNew;
      
      
        //----------- 1st Successor (North) ------------
 
        // Only process this cell if this is a valid one
        if (isValid(i-1, j) == true)
        {
               // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j, dest) == true)
            {
                  // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j] == false &&
                     isUnBlocked(grid, i-1, j) == true)
            {
                  gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i-1, j, dest);
                fNew = gNew + hNew;
 
                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j].f == FLT_MAX ||
                        cellDetails[i-1][j].f > fNew)
                {
                    insert( make_pair2(fNew,make_pair1(i-1, j)));
 
                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }
 
        //----------- 2nd Successor (South) ------------
 
        // Only process this cell if this is a valid one
        if (isValid(i+1, j) == true)
        {
          
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j, dest) == true)
            {
              
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(grid, i+1, j) == true)
            {
              
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest);
                fNew = gNew + hNew;
 
                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i+1][j].f == FLT_MAX ||
                        cellDetails[i+1][j].f > fNew)
                {
                    ;
                    insert( make_pair2(fNew, make_pair1 (i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }
 
        //----------- 3rd Successor (East) ------------
 
        // Only process this cell if this is a valid one
        if (isValid (i, j+1) == true)
        {
          
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j+1, dest) == true)
            {
              
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
 
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j+1] == false &&
                     isUnBlocked (grid, i, j+1) == true)
            {
              
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest);
                fNew = gNew + hNew;
 
                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j+1].f == FLT_MAX ||
                        cellDetails[i][j+1].f > fNew)
                {
                  
                    insert( make_pair2(fNew,
                                        make_pair1(i, j+1)));
 
                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }
 
        //----------- 4th Successor (West) ------------
 
        // Only process this cell if this is a valid one
        if (isValid(i, j-1) == true)
        {
          
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j-1, dest) == true)
            {
              
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
 
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j-1] == false &&
                     isUnBlocked(grid, i, j-1) == true)
            {
              
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest);
                fNew = gNew + hNew;
 
                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j-1].f == FLT_MAX ||
                        cellDetails[i][j-1].f > fNew)
                {
                  
                    insert( make_pair2(fNew,
                                          make_pair1(i, j-1)));
 
                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }
    }
 
    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
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
void right(void)
{
    stop();
    _delay_ms(100);
    velocity(200,200);
    forward_mm(63);
    right_degrees(70);
    motion_set(0x0A);
    velocity(190,190);
    while (!(C>0x0D && L<0x0D && R<0x0D))
    {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor
    }
  stop();
  _delay_ms(200);
  return;
}
void left(void)
{
    stop();
    _delay_ms(100);
    velocity(200,200);
    forward_mm(63);
    left_degrees(70);
    motion_set(0x05);
    velocity(190,190);  
  while (!(C>0x0D && L<0x0D && R<0x0D))
  {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor
      
  }
  stop();
  _delay_ms(200);
  return;
}
void reverse(void)
{
    stop();
	forward_mm(70);
	stop();
    motion_set(0x0A);
    velocity(150,150);
    _delay_ms(2000);
    while (!(C<0x0D && L<0x0D && R>0x0D))
    {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R= ADC_Conversion(1);    //Getting data of Right WL Sensor
      
    }
  stop();
  _delay_ms(200);
  return;
}
void orient_reverse(void)
{
    stop();
    velocity(185,185);
    forward_mm(70);
    right_degrees(170);
	motion_set(0x0A);
    while (!(C>0x0D && L<0x0D && R<0x0D))
    {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor
    }
	stop();
	_delay_ms(300);
    motion_set(0x09);
    velocity(100,100);
    while(1)
    {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor

        if((C>0x0D && L>0x0D && R<0x0D)||(C>0x0D && L>0x0D && R>0x0D)||(C>0x0D && L<0x0D && R>0x0D))
        {
            backflag=1;
            forward();
            _delay_ms(200);
            break;
        }
    }  
stop();
motion_set(0x0a);
velocity(120,120);
_delay_ms(50);
stop();
}
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
                velocity(145,145);
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
               velocity(145,145);
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
               velocity(145,145);
                back_mm(60);
                return;
            }
            else if(direction1==3)
            {
                right();
                updirect(0);
                motion_set(0x09);
               velocity(145,145);
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
               velocity(145,145);
                back_mm(60);
                return;
            }
            else if(direction1==2)
            {
                right();
                updirect(0);
                motion_set(0x09);
               velocity(145,145);
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
               velocity(145,145);
                back_mm(60);
                return;
            }
            else if(direction1==1)
            {
                reverse();
                updirect(3);
                motion_set(0x09);
               velocity(145,145);
                back_mm(25);
                return;
            }
            else if(direction1==2)
            {
                left();
                updirect(2);
                motion_set(0x09);
               velocity(145,145);
                back_mm(60);
                return;
            }
        }
    }
    return;
}


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


int deposition_zone_allign(int d,int grid[][COL])
{  
    int i;
    if(d==1)
    {     
        for(i=0;i<3;i++)
        {
            if(isUnBlocked(grid,apple_deposition_zones[i]/7,apple_deposition_zones[i]%7))
            {
                return(i);
            }
        }
    }
    else if(d==2)
    {
        for(i=0;i<3;i++)
        {
            if(isUnBlocked(grid,blueberry_deposition_zones[i]/7,blueberry_deposition_zones[i]%7))
            {
                return(i);
            }
        }
    }
   else if(d==3)
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


void deposit(int d)
{	int i;
    tree_count++;
    if(d==0)
    {
        if(direction1==0)
        {
            forward_mm(60);
            right_degrees(135);
            _delay_ms(500);
			for(i=0;i<=110;i++)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
			for(i=110;i>=0;i--)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
            left_degrees(135);
            motion_set(0x05);
        }
        else if(direction1==1)
        {
            forward_mm(60);
            left_degrees(135);
            _delay_ms(500);
			for(i=0;i<=110;i++)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
			for(i=110;i>=0;i--)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
            right_degrees(135);
            motion_set(0x0A);
        }
      
        else if(direction1==2)
        {
            forward_mm(60);
            left_degrees(45);
            _delay_ms(500);
			//servo_4(110);
			for(i=0;i<=110;i++)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
				servo_4(i);
				_delay_ms(8);
			}
            _delay_ms(500);
            right_degrees(45);
            motion_set(0x0A);
        }
        else if(direction1==3)
        {
            forward_mm(60);
            right_degrees(45);

            _delay_ms(500);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(45);
            motion_set(0x05);
          
            //node detection
        }
      
    }
  
    else if(d==1)
    {
        if(direction1==0)
        {
            forward_mm(60);
            right_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
            left_degrees(45);
            motion_set(0x05);
          
          
            //node detection
          
        }
        else if(direction1==1)
        {
            forward_mm(60);
            right_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(135);
            motion_set(0x05);
          
          
            //node detection
        }
      
        else if(direction1==2)
        {
            forward_mm(60);
            left_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            right_degrees(135);
            motion_set(0x05);
          
          
            //node detection
        }
        else if(direction1==3)
        {
            forward_mm(60);
            left_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            right_degrees(45);
            motion_set(0x0A);
          
            //node detection
        }
    }
      
      
    else if(d==3)
    {
        if(direction1==0)
        {
            forward_mm(60);
            left_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            right_degrees(45);
            motion_set(0x0A);
          
          
            //node detection
          
        }
        else if(direction1==1)
        {
            forward_mm(60);
            right_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(45);
            motion_set(0x05);
          
          
            //node detection
        }
      
        else if(direction1==2)
        {
            forward_mm(60);
            right_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(135);
            motion_set(0x05);
          
          
            //node detection
        }
        else if(direction1==3)
        {
            forward_mm(60);
            left_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
          
            right_degrees(135);
            motion_set(0x0A);
          
            //node detection
        }
    }
      
    else if(d==2)
    {
        if(direction1==0)
        {
            forward_mm(60);
            left_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            right_degrees(135);
            motion_set(0x0A);
          
          
            //node detection
          
        }
        else if(direction1==1)
        {
            forward_mm(60);
            left_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            right_degrees(45);
            motion_set(0x0A);
          
          
            //node detection
        }
      
        else if(direction1==2)
        {
            forward_mm(60);
            right_degrees(45);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(45);
            motion_set(0x05);
          
          
            //node detection
        }
        else if(direction1==3)
        {
            forward_mm(60);
            right_degrees(135);

            _delay_ms(1000);

			//servo_4(110);
			for(i=0;i<=110;i++)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1500);
            //servo_4(0);
			for(i=110;i>=0;i--)
			{
			servo_4(i);
			_delay_ms(8);
			}
            _delay_ms(1000);
          
          
            left_degrees(135);
            motion_set(0x05);
          
            //node detection
        }
    }
    /*  
    velocity(100,100);
    while (!(C>0x0D && L<0x0D && R<0x0D))
    {
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor
    }*/
	stop();
    return;
}

void flag_update_if_not_changed()                   
{
    if(fruit_recognized_flag!=1)
    {
        zoom=tree_flag;
        if(zoom!=0)
        {
            fruit_recognized_flag=1;
        }
    }
}
  
 void fruitpluck(void)                                                                
{
    orientbot();
    int pi_dis=0;
	int fp=0;
    float i,t;
    lcd_print(2,1,currnode[0],1);
    lcd_print(2,2,currnode[1],1);
    lcd_print(2,3,direction1,1);
    servo_1(40);
    servo_2(218);
    servo_3(50);//open hand
	_delay_ms(2000);
    PORTD=  0x20; //pi enable     
    for(i=218;i>185;i=i-1)
    {
        servo_2(i);
        _delay_ms(250);
        pi_dis=PIND & 0x10; // take pi_disable input;
        if(pi_dis)
        {
			PORTD =  0x00;
			_delay_ms(250);
			PORTD = 0x60;     //fruit present pin and enable pin is made high PD6 and PD5
			i=0;
			while(!(fp || i==70))
			{
				_delay_ms(50);
				fp= PIND & 0x80; 
				i++;
			}		
			
			     // fruit pluck pin ie., PD7
			if(fp)
			{
				tree_flag= PIND & 0x03;
				PORTD =  0x00;
				flag_update_if_not_changed();
				for(i=50;i>=15;i--)
				{
					servo_1(i);
					_delay_ms(20);
				}
				servo_2(200);
				_delay_ms(500);
				for(i=30;i<=100;i++)
				{
					servo_3(i);
					_delay_ms(10);
				}
				t=203;
				for(i=15;i<=50;i++)
				{
					servo_1(i);
					servo_2(t);
					t++;
					_delay_ms(50);
				}
				for(i=50;i>=25;i=i-2)
				{
					servo_1(i);
					servo_2(t);
					t--;
					_delay_ms(40);
				}
				for(i=t;i>=25;i--)
				{
					_delay_ms(10);
					servo_2(i);
				}
				for(i=25;i>0;i--)
				{
					servo_1(i);
					_delay_ms(20);
				}
				for(i=100;i>=70;i--)
				{
					servo_3(i);
				}
				_delay_ms(100);
				for(i=0;i<=20;i++)
				{
					servo_1(i);
					_delay_ms(30);
				}
				for(i=15;i<=218;i++)
				{
					servo_2(i);
				}
				for(i=20;i<=50;i++)
				{
					servo_1(i);
					_delay_ms(30);
				}
				_delay_ms(500);
				servo_3_free();
				break;
			}
		}
    }  
	servo_1(50);
	servo_2(190);
    tree_flag= PIND & 0x03;
    PORTD =  0x00;
    flag_update_if_not_changed();
    return;
}
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
    else                                                                               ///////////final destination reached
    {
        forward();               
        //_delay_ms(1000);
        if((cpoint==count*2)&&(fruitconf==0) )                                                                
        {
            stop();
            _delay_ms(500);
            valid_deposition_node=deposition_zone_allign(zoom,grid);
            deposit(valid_deposition_node);
            _delay_ms(500);
            exit1=1;
            fruit_recognized_flag=0;
            return;
        }
        else if(fruitconf==1)
        {
            stop();
            fruitpluck();
            _delay_ms(500);
            exit1=1;
            return;
        }
    }
}
void traverse(int grid[][COL])
{
    while(1)
    {
	servo_1(50);
        L = ADC_Conversion(3);    //Getting data of Left WL Sensor
        C = ADC_Conversion(2);    //Getting data of Center WL Sensor
        R = ADC_Conversion(1);    //Getting data of Right WL Sensor

        if(backflag==0)
        {
            motion_set(0x09);
            velocity(100,100);
            while(1)
            {
                L = ADC_Conversion(3);    //Getting data of Left WL Sensor
                C = ADC_Conversion(2);    //Getting data of Center WL Sensor
                R = ADC_Conversion(1);    //Getting data of Right WL Sensor
 
                if((C>0x0D && L>0x0D && R<0x0D)||(C>0x0D && L>0x0D && R>0x0D)||(C>0x0D && L<0x0D && R>0x0D))
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
        if(C>0x0D)
        {
            flag=1;
            forward();
            velocity(255,255);
        }
        if((L<0x0D) && (flag==0))
        {
            flag=1;
            forward();
            velocity(180,90);
        }
        if((R<0x0D) && (flag==0))
        {
            flag=1;
            forward();
            velocity(90,180);
        }
        if(C<0x0D && L<0x0D && R<0x0D)
        {
            forward();
            velocity(220,220);

        }
        if((C>0x0D && L>0x0D && R<0x0D)||(C>0x0D && L>0x0D && R>0x0D)||(C>0x0D && L<0x0D && R>0x0D))
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
                        printf("left ");
                        updirect(2);
                    }
                    else if(direction1==1)
                    {
                        forward();
                        printf("forward ");
                        _delay_ms(300);
                    }
                    else if(direction1==2)
                    {
                      
                        right();
                        printf("right ");
                        updirect(0);
                    }
                    else if(direction1==3)
                    {
                        reverse();
                        printf("reverse ");
                        updirect(3);
                    }
                }
                if(currnode[1]>nextnode[1])
                {
                    if (direction1==0)
                    {
                        right();
                        printf("right ");
                        updirect(0);
                    }
                    else if(direction1==1)
                    {
                        reverse();
                        printf("reverse ");
                        updirect(3);
                    }
                    else if(direction1==2)
                    {
                        left();
                        printf("left ");
                        updirect(2);
                    }
                    else if(direction1==3)
                    {
                        forward();
                        printf("forward ");
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
                        printf("reverse ");
                        updirect(3);
                    }
                    else if(direction1==1)
                    {
                        left();
                        printf("left ");
                        updirect(2);
                    }
                    else if(direction1==2)
                    {
                        forward();
                        printf("forward ");
                        _delay_ms(300);
                    }
                    else if(direction1==3)
                    {
                        right();
                        printf("right ");
                        updirect(0);
                    }
                }
                if(currnode[0]>nextnode[0])
                {
                    if (direction1==0)
                    {
                        forward();
                        printf("forward ");
                        _delay_ms(300);
                    }
                    else if(direction1==1)
                    {
                        right();
                        printf("right ");
                        updirect(0);
                    }
                    else if(direction1==2)
                    {
                        reverse();
                        printf("reverse ");
                        updirect(3);
                    }
                    else if(direction1==3)
                    {
                        left();
                        printf("left ");
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
    struct Pair src;                    //source pair
    struct Pair dest;                   //destination pair
    /* Description of the Grid-
     1--> The cell is not blocked
     --> The cell is blocked    */
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
	servo_1(50);
	servo_2(218);
  	servo_3(50);
	servo_4(0);
    int x,y,i,j;            // The below procedure is to place the trres in the map and make them recognized as obstacle
    for(i=0;i<TREES;i++)
    {
      x=tree[i]/7;
      y=tree[i]%7;
      if(isValid(x,y))
      {
          grid[x][y]=0;
      }
    }
  
    dep_assign(grid);

    for(i=0;i<7;i++)                        //printing the map ......................can delete this in firebird
    {
      for(j=0;j<7;j++)
      {
        printf("%d,",grid[i][j]);
      }
      printf("\n");
    }
  
     printf("\n");
  
  
    int c=0;                                //indexing variables
    for(i=0;i<TREES;i++)
    {
      x=tree[i]/7;                             //checking all the adjacent nodes of (x,y)
      y=tree[i]%7;
      if(isValid(x-1,y) && isUnBlocked(grid,x-1,y))
      {
          printf("%d,%d is valid\n",x-1,y);                // the south node
          desti[c]=x-1;                                       //Destination array is the array of max length 80 that take x,y values of valid path continuously
          c++;                                                 //c is the indexing variable used to have a count of value till desti reaches
          desti[c]=y;                                          //eg: desti=[1,2,3,4,5,6],,then c is at 5 ie desti[5] is last element
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
      
/*      
        temp=dep[e];                        //The procedure below is done to accomodate deposition zones...dep consists of deposition nodes at the start of main()
                                              
        desti[c]=temp/7;                    //deposition zone...x coordinate
        dep_c[d]=desti[c];
        c++;
        d++;
      
      
        desti[c]=temp%7;                        //deposition zone.....y cocordinate
        dep_c[d]=desti[c];                      //Updating the deposition zones for further calculations
  
        c++;
        d++;
      
        e++;                               //considering the next deposition zone number
      
*/                                           //dep_c consists of coordinates of destination nodes
    desti[c]=1000;
    c++;
    desti[c]=1000;
    c++;
    }
  
  
    int count;
    count=c;
    for(i=0;i<count-1;i+=2)
    {
      printf("(%d,%d),",desti[i],desti[i+1]);
    }
  
    src = make_pair1(0, 0);
  
   //////////////////////////////////////////////////////////CHANGE FROM HERE...just copy everyrthing in MAIN ...dont change anything
    for(i=0;i<count-1;i+=2)
    {
      if(desti[i]==1000 && desti[i+1]==1000)
      {
         dest=find_dep_zone();
        fruitconf=0;
      }
      else
      {
            dest= make_pair1(desti[i],desti[i+1]);
            fruitconf=1;
      }
    
      if(!isValid(dest.first,dest.second))
          continue;
    
       lcd_print(2,1,0,1);
       lcd_print(2,2,0,1);
       lcd_print(2,3,0,1);
      aStarSearch(grid,src,dest);
           traverse(grid);
      //////////////create the function and write return();
      //////////////////////////////////////////////////////////////////////////////FUNCTION
      //src=make_pair1(desti[i],desti[i+1]);
      src=make_pair1(dest.first,dest.second);
      printf("\n\n");

    }
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
    return(0);
}

